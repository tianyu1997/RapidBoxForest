#include <LECT/evidence_store.h>
#include <LECT/mmap_lect_file.h>

#include "detail/float16_codec.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

using lect::detail::f32_to_f16;
using lect::detail::f16_to_f32;
using lect::detail::encode_f16;
using lect::detail::decode_f16;

namespace lect {

// ─── FlatPayloadFile ──────────────────────────────────────────────────────────
// Fixed-stride binary format for disk-backed evidence.
// Header (32 bytes) | active_link_radii (float32[nA]) | N×2 records.
// Record for (node, ch): [flags:u8][pad:3][ep_f16:nA*24B]
// Flat payload persists endpoint evidence only; link envelopes are re-derived on demand.
// Flag byte: bit0=has_endpoint, bit1=has_any_envelope, bits[4:2]=EndpointSource.
static constexpr uint32_t kFlatPayloadMagic   = 0x50464C43u;  // "CFLP"
static constexpr uint32_t kFlatPayloadVersion = 2u;

struct FlatPayloadFileHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t n_active_links;
    uint32_t n_channels;
    uint64_t n_nodes;
    uint32_t stride;       // bytes per (node, channel) record
    uint32_t data_offset;  // = 32 + n_active_links * 4
};
static_assert(sizeof(FlatPayloadFileHeader) == 32, "header must be 32 bytes");

struct EvidenceStore::FlatPayloadFile {
    MappedLectFile mapped;
    std::filesystem::path path;
    uint32_t n_active_links = 0;
    uint64_t n_nodes        = 0;
    uint32_t stride         = 0;
    uint32_t data_offset    = 0;
    std::vector<float> active_link_radii;

    bool valid() const { return mapped.is_open(); }

    const uint8_t* record_ptr(int node, int ch_idx) const {
        return reinterpret_cast<const uint8_t*>(mapped.bytes().data())
               + data_offset
               + (static_cast<std::size_t>(node) * kNumChannels + ch_idx) * stride;
    }

    // Decode endpoint_f16 from a record into float32.
    std::vector<float> read_endpoint(int node, int ch_idx) const {
        const uint8_t* r = record_ptr(node, ch_idx);
        if (!(r[0] & 0x01u)) return {};
        const int n = static_cast<int>(n_active_links) * 2 * 6;
        const auto* f16 = reinterpret_cast<const uint16_t*>(r + 4);
        std::vector<float> out(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) out[i] = f16_to_f32(f16[i]);
        return out;
    }

    // Build a StoredLinkAabb from a record.
    EvidenceStore::StoredLinkAabb read_link_aabb(int node, int ch_idx) const {
        const uint8_t* r = record_ptr(node, ch_idx);
        if (!(r[0] & 0x01u)) return {};
        const int nA = static_cast<int>(n_active_links);
        const auto endpoint = read_endpoint(node, ch_idx);
        if (endpoint.empty()) return {};
        std::vector<float> link_iaabbs(static_cast<std::size_t>(nA) * 6, 0.0f);
        for (int link_idx = 0; link_idx < nA; ++link_idx) {
            const int prox = (link_idx * 2) * 6;
            const int dist = (link_idx * 2 + 1) * 6;
            const int out = link_idx * 6;
            for (int axis = 0; axis < 3; ++axis) {
                link_iaabbs[static_cast<std::size_t>(out + axis)] =
                    std::min(endpoint[static_cast<std::size_t>(prox + axis)],
                             endpoint[static_cast<std::size_t>(dist + axis)]);
                link_iaabbs[static_cast<std::size_t>(out + axis + 3)] =
                    std::max(endpoint[static_cast<std::size_t>(prox + axis + 3)],
                             endpoint[static_cast<std::size_t>(dist + axis + 3)]);
            }
        }
        EvidenceStore::StoredLinkAabb aabb;
        aabb.n_active_links   = nA;
        aabb.n_subdivisions   = 1;
        aabb.active_link_radii = active_link_radii;
        aabb.link_iaabbs_f16 = encode_f16(link_iaabbs);
        return aabb;
    }

    // Decode the link AABBs as float32 (same data as StoredLinkAabb but decoded).
    std::vector<float> read_link_aabbs_f32(int node, int ch_idx) const {
        return decode_f16(read_link_aabb(node, ch_idx).link_iaabbs_f16);
    }
};

// Destructor: defined here where FlatPayloadFile is complete.
EvidenceStore::~EvidenceStore() = default;
EvidenceStore::EvidenceStore(EvidenceStore&&) noexcept = default;
EvidenceStore& EvidenceStore::operator=(EvidenceStore&&) noexcept = default;

// ─── Flag helpers (file-local) ────────────────────────────────────────────────
static constexpr uint8_t kFlagHasEndpoint = 0x01u;
static constexpr uint8_t kFlagHasEnvelope = 0x02u;
static constexpr uint8_t kFlagHasAny      = kFlagHasEndpoint | kFlagHasEnvelope;
static constexpr uint8_t kFlagSrcShift    = 2u;
static constexpr uint8_t kFlagSrcMask     = 0x1Cu;

static inline uint8_t make_flag(bool has_ep, bool has_env, rbf::EndpointSource src) noexcept {
    return (has_ep ? kFlagHasEndpoint : 0u)
         | (has_env ? kFlagHasEnvelope : 0u)
         | ((static_cast<uint8_t>(src) & 0x07u) << kFlagSrcShift);
}
static inline rbf::EndpointSource flag_source(uint8_t f) noexcept {
    return static_cast<rbf::EndpointSource>((f >> kFlagSrcShift) & 0x07u);
}
static inline int flag_ci(Channel ch) noexcept { return channel_index(ch); }

EvidenceStore::EvidenceStore(Layout layout, int n_nodes) {
    reset(layout, n_nodes);
}

void EvidenceStore::reset(Layout layout, int n_nodes) {
    if (layout.n_active_links < 0) {
        throw std::invalid_argument("LECT evidence layout has negative active link count");
    }
    layout_ = layout;
    flat_payload_cutoff_ = 0;
    flat_payload_.reset();
    node_channel_flags_.clear();
    nodes_.clear();
    resize_nodes(n_nodes);
}

EvidenceStore::EvidenceStore(const EvidenceStore& other)
        : node_channel_flags_(other.node_channel_flags_),
            flat_payload_cutoff_(other.flat_payload_cutoff_),
            layout_(other.layout_) {
    nodes_.resize(other.nodes_.size());
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
        if (other.nodes_[i])
            nodes_[i] = std::make_unique<NodeEvidence>(*other.nodes_[i]);
    }
    if (other.flat_payload_ && other.flat_payload_->valid() && !other.flat_payload_->path.empty()) {
        attach_flat_payload(other.flat_payload_->path);
    }
}

EvidenceStore& EvidenceStore::operator=(const EvidenceStore& other) {
    if (this != &other) {
        layout_ = other.layout_;
        node_channel_flags_ = other.node_channel_flags_;
        flat_payload_cutoff_ = other.flat_payload_cutoff_;
        flat_payload_.reset();
        nodes_.resize(other.nodes_.size());
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            if (other.nodes_[i])
                nodes_[i] = std::make_unique<NodeEvidence>(*other.nodes_[i]);
            else
                nodes_[i].reset();
        }
        if (other.flat_payload_ && other.flat_payload_->valid() && !other.flat_payload_->path.empty()) {
            attach_flat_payload(other.flat_payload_->path);
        }
    }
    return *this;
}

void EvidenceStore::resize_nodes(int n_nodes) {
    if (n_nodes < 0) {
        throw std::invalid_argument("LECT evidence node count must be non-negative");
    }
    const int old_nodes = static_cast<int>(nodes_.size());
    // Lazy: only grow the pointer vector (8 B/entry); NodeEvidence allocated on first write.
    nodes_.resize(static_cast<std::size_t>(n_nodes));
    // Grow hot flags array with zeros (no evidence).
    node_channel_flags_.resize(static_cast<std::size_t>(n_nodes) * kNumChannels, 0u);
    if (flat_payload_) {
        flat_payload_cutoff_ = static_cast<int>(std::min<std::uint64_t>(
            flat_payload_->n_nodes, static_cast<std::uint64_t>(n_nodes)));
    }
    hydrate_flat_payload_flags(old_nodes, n_nodes);
}

void EvidenceStore::copy_node_from(int dst_node, const EvidenceStore& source, int src_node) {
    require_node(dst_node);
    source.require_node(src_node);
    if (layout_.n_active_links != source.layout_.n_active_links) {
        throw std::invalid_argument("LECT evidence layouts differ during copy");
    }
    nodes_[dst_node].reset();
    for (int ci = 0; ci < kNumChannels; ++ci) {
        node_channel_flags_[dst_node * kNumChannels + ci] = 0u;
    }
    for (Channel channel : {Channel::Safe, Channel::Unsafe}) {
        const bool has_endpoint = source.has_endpoint(src_node, channel);
        const bool has_link = source.has_link_aabbs(src_node, channel);
        const bool has_envelope = source.has_any_envelope(src_node, channel);
        if (!has_endpoint && !has_link && !has_envelope) {
            continue;
        }
        const auto src = (has_endpoint || has_envelope)
            ? source.source(src_node, channel)
            : rbf::EndpointSource::IFK;
        if (has_endpoint) {
            set_endpoint(dst_node, channel, src,
                         source.endpoint(src_node, channel),
                         source.quality_meta(src_node, channel));
        }
        if (has_link) {
            set_link_aabbs(dst_node, channel, source.link_aabbs(src_node, channel));
        }
        if (has_envelope) {
            set_typed_envelope(dst_node, channel, src,
                               source.reconstruct_link_envelope(src_node, channel));
        }
    }
}

void EvidenceStore::set_endpoint(int node, Channel channel, rbf::EndpointSource source,
                                 std::vector<float> endpoint_iaabbs,
                                 rbf::SourceQualityMetadata quality_meta) {
    require_node(node);
    require_channel(channel);
    require_endpoint_payload(endpoint_iaabbs);
    if (!nodes_[node]) nodes_[node] = std::make_unique<NodeEvidence>();

    auto& slot = nodes_[node]->channels[channel_index(channel)];
    slot.has = true;
    slot.source = source;
    slot.quality_meta = quality_meta;
    slot.endpoint_f16 = encode_f16(endpoint_iaabbs);
    slot.has_link_aabb = false;
    slot.link_aabbs.clear();
    slot.link_aabb_envelope.reset();
    slot.link_kdop_envelope.reset();
    slot.link_sh_envelope.reset();
    // Sync hot flags.
    uint8_t& f = node_channel_flags_[node * kNumChannels + channel_index(channel)];
        f = kFlagHasEndpoint
      | ((static_cast<uint8_t>(source) & 0x07u) << kFlagSrcShift);
}

void EvidenceStore::set_link_aabbs(int node, Channel channel, std::vector<float> link_aabbs) {
    require_node(node);
    require_channel(channel);
    require_link_payload(link_aabbs);
    if (!nodes_[node]) nodes_[node] = std::make_unique<NodeEvidence>();

    auto& slot = nodes_[node]->channels[channel_index(channel)];
    slot.has_link_aabb = true;
    slot.link_aabbs = std::move(link_aabbs);
}

void EvidenceStore::clear_channel_evidence(int node, Channel channel) {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    if (nodes_[node]) {
        nodes_[node]->channels[ci] = ChannelEvidence{};
        bool has_any_channel = false;
        for (const auto& slot : nodes_[node]->channels) {
            has_any_channel = has_any_channel || slot.has || slot.has_link_aabb ||
                slot.link_aabb_envelope.has_value() || slot.link_kdop_envelope.has_value() ||
                slot.link_sh_envelope.has_value();
        }
        if (!has_any_channel) {
            nodes_[node].reset();
        }
    }
    node_channel_flags_[node * kNumChannels + ci] = 0u;
}

void EvidenceStore::clear_node_evidence(int node) {
    require_node(node);
    nodes_[node].reset();
    for (int ci = 0; ci < kNumChannels; ++ci) {
        node_channel_flags_[node * kNumChannels + ci] = 0u;
    }
}

void EvidenceStore::set_link_aabb_envelope(int node, Channel channel,
                                           rbf::EndpointSource source,
                                           StoredLinkAabb aabb) {
    require_node(node);
    require_channel(channel);
    aabb.active_link_radii.assign(static_cast<std::size_t>(std::max(0, aabb.n_active_links)), 0.0f);
    if (!nodes_[node]) nodes_[node] = std::make_unique<NodeEvidence>();
    auto& slot = nodes_[node]->channels[channel_index(channel)];
    slot.source = source;
    slot.link_aabb_envelope = std::move(aabb);
    // Sync hot flags.
    uint8_t& f = node_channel_flags_[node * kNumChannels + channel_index(channel)];
    f = (f & ~kFlagSrcMask)
      | kFlagHasEnvelope
      | ((static_cast<uint8_t>(source) & 0x07u) << kFlagSrcShift);
}

void EvidenceStore::set_link_kdop_envelope(int node, Channel channel,
                                           rbf::EndpointSource source,
                                           StoredLinkKDop kdop) {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) nodes_[node] = std::make_unique<NodeEvidence>();
    auto& slot = nodes_[node]->channels[channel_index(channel)];
    slot.source = source;
    slot.link_kdop_envelope = std::move(kdop);
    uint8_t& f = node_channel_flags_[node * kNumChannels + channel_index(channel)];
    f = (f & ~kFlagSrcMask)
      | kFlagHasEnvelope
      | ((static_cast<uint8_t>(source) & 0x07u) << kFlagSrcShift);
}

void EvidenceStore::set_link_sh_envelope(int node, Channel channel,
                                         rbf::EndpointSource source,
                                         StoredLinkSupportHull sh) {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) nodes_[node] = std::make_unique<NodeEvidence>();
    auto& slot = nodes_[node]->channels[channel_index(channel)];
    slot.source = source;
    slot.link_sh_envelope = std::move(sh);
    uint8_t& f = node_channel_flags_[node * kNumChannels + channel_index(channel)];
    f = (f & ~kFlagSrcMask)
      | kFlagHasEnvelope
      | ((static_cast<uint8_t>(source) & 0x07u) << kFlagSrcShift);
}

void EvidenceStore::set_typed_envelope(int node, Channel channel,
                                       rbf::EndpointSource source,
                                       const rbf::LinkEnvelope& envelope) {
    require_node(node);
    require_channel(channel);
    // AABB fields are the base — always stored for any envelope type.
    if (!envelope.link_iaabbs.empty()) {
        StoredLinkAabb aabb;
        aabb.n_active_links = envelope.n_active_links;
        aabb.n_subdivisions = envelope.n_subdivisions;
        aabb.active_link_radii = envelope.active_link_radii;
        aabb.link_iaabbs_f16 = encode_f16(envelope.link_iaabbs);
        set_link_aabb_envelope(node, channel, source, std::move(aabb));
    }
    // KDop: present only for KDOP envelopes.
    if (!envelope.kdop_intervals.empty()) {
        StoredLinkKDop kdop;
        kdop.n_active_links = envelope.n_active_links;
        kdop.n_subdivisions = envelope.n_subdivisions;
        kdop.kdop_n_axes = envelope.kdop_n_axes;
        kdop.kdop_direction_set = envelope.kdop_direction_set;
        kdop.kdop_intervals = envelope.kdop_intervals;
        set_link_kdop_envelope(node, channel, source, std::move(kdop));
    }
    // SupportHull slot.
    if (!envelope.support_hulls.empty()) {
        StoredLinkSupportHull sh;
        sh.n_active_links = envelope.n_active_links;
        sh.n_subdivisions = envelope.n_subdivisions;
        sh.support_hulls = envelope.support_hulls;
        set_link_sh_envelope(node, channel, source, std::move(sh));
    }
}

bool EvidenceStore::has_endpoint(int node, Channel channel) const {
    require_node(node);
    return (node_channel_flags_[node * kNumChannels + channel_index(channel)] & kFlagHasEndpoint) != 0;
}

bool EvidenceStore::has_link_aabbs(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    if (nodes_[node] && nodes_[node]->channels[ci].has_link_aabb) return true;
    return false;
}

bool EvidenceStore::has_link_aabb_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    if (nodes_[node] && nodes_[node]->channels[ci].link_aabb_envelope.has_value()) return true;
    return false;
}

bool EvidenceStore::has_link_kdop_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) return false;
    return nodes_[node]->channels[channel_index(channel)].link_kdop_envelope.has_value();
}

bool EvidenceStore::has_link_sh_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) return false;
    return nodes_[node]->channels[channel_index(channel)].link_sh_envelope.has_value();
}

bool EvidenceStore::has_any_envelope(int node, Channel channel) const {
    require_node(node);
    if (!nodes_[node] && node < flat_payload_cutoff_ && flat_payload_) {
        return false;
    }
    return (node_channel_flags_[node * kNumChannels + channel_index(channel)] & kFlagHasEnvelope) != 0;
}

rbf::EndpointSource EvidenceStore::source(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const uint8_t f = node_channel_flags_[node * kNumChannels + channel_index(channel)];
    if (!(f & kFlagHasAny)) throw std::logic_error("LECT evidence source is not present");
    return flag_source(f);
}

rbf::SourceQualityMetadata EvidenceStore::quality_meta(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) return {};
    return nodes_[node]->channels[channel_index(channel)].quality_meta;
}

EvidenceStore::ChannelInfo EvidenceStore::channel_info(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    const uint8_t flags = node_channel_flags_[node * kNumChannels + ci];
    ChannelInfo info;
    info.has = (flags & kFlagHasEndpoint) != 0;
    info.source = (flags & kFlagHasAny) ? flag_source(flags) : rbf::EndpointSource::IFK;
    if (!nodes_[node]) return info;
    const auto& slot = nodes_[node]->channels[ci];
    const bool slot_has_any = slot.has || slot.has_link_aabb ||
        slot.link_aabb_envelope.has_value() || slot.link_kdop_envelope.has_value() ||
        slot.link_sh_envelope.has_value();
    if (!slot_has_any) return info;
    info.has = slot.has;
    info.has_link_aabb = slot.has_link_aabb;
    info.has_link_aabb_envelope = slot.link_aabb_envelope.has_value();
    info.has_link_kdop_envelope = slot.link_kdop_envelope.has_value();
    info.has_link_sh_envelope = slot.link_sh_envelope.has_value();
    info.source = slot.source;
    return info;
}

std::vector<float> EvidenceStore::endpoint(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    if (nodes_[node]) {
        const auto& slot = nodes_[node]->channels[ci];
        if (slot.has) {
            return decode_f16(slot.endpoint_f16);
        }
    }
    if (node < flat_payload_cutoff_ && flat_payload_)
        return flat_payload_->read_endpoint(node, ci);
    throw std::logic_error("LECT endpoint evidence is not present");
}

const std::vector<float>& EvidenceStore::link_aabbs(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    static thread_local std::vector<float> tl_aabbs;
    if (nodes_[node]) {
        const auto& slot = nodes_[node]->channels[ci];
        if (slot.has_link_aabb) {
            return slot.link_aabbs;
        }
        if (slot.link_aabb_envelope) {
            tl_aabbs = decode_f16(slot.link_aabb_envelope->link_iaabbs_f16);
            return tl_aabbs;
        }
    }
    if (node < flat_payload_cutoff_ && flat_payload_) {
        tl_aabbs = flat_payload_->read_link_aabbs_f32(node, ci);
        if (!tl_aabbs.empty()) return tl_aabbs;
        const auto stored_aabb = flat_payload_->read_link_aabb(node, ci);
        if (stored_aabb.n_active_links > 0 && !stored_aabb.link_iaabbs_f16.empty()) {
            tl_aabbs = decode_f16(stored_aabb.link_iaabbs_f16);
            return tl_aabbs;
        }
    }
    if (has_endpoint(node, channel)) {
        tl_aabbs = derive_link_aabbs(node, channel, {});
        return tl_aabbs;
    }
    throw std::logic_error("LECT stored link AABB evidence is not present");
}

const EvidenceStore::StoredLinkAabb& EvidenceStore::link_aabb_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    const int ci = channel_index(channel);
    if (nodes_[node]) {
        const auto& slot = nodes_[node]->channels[ci];
        if (slot.link_aabb_envelope) {
            return *slot.link_aabb_envelope;
        }
    }
    if (node < flat_payload_cutoff_ && flat_payload_) {
        static thread_local StoredLinkAabb tl_aabb;
        tl_aabb = flat_payload_->read_link_aabb(node, ci);
        if (tl_aabb.n_active_links > 0) return tl_aabb;
    }
    throw std::logic_error("LECT stored link-AABB envelope is not present");
}

const EvidenceStore::StoredLinkKDop& EvidenceStore::link_kdop_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) throw std::logic_error("LECT stored link-KDop envelope is not present");
    const auto& slot = nodes_[node]->channels[channel_index(channel)];
    if (!slot.link_kdop_envelope) {
        throw std::logic_error("LECT stored link-KDop envelope is not present");
    }
    return *slot.link_kdop_envelope;
}

const EvidenceStore::StoredLinkSupportHull& EvidenceStore::link_sh_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);
    if (!nodes_[node]) throw std::logic_error("LECT stored link-SupportHull envelope is not present");
    const auto& slot = nodes_[node]->channels[channel_index(channel)];
    if (!slot.link_sh_envelope) {
        throw std::logic_error("LECT stored link-SupportHull envelope is not present");
    }
    return *slot.link_sh_envelope;
}

rbf::LinkEnvelope EvidenceStore::reconstruct_link_envelope(int node, Channel channel) const {
    require_node(node);
    require_channel(channel);

    const int ci = channel_index(channel);

    auto reconstruct_flat = [&]() -> rbf::LinkEnvelope {
        const uint8_t f = node_channel_flags_[node * kNumChannels + ci];
        if (!(f & kFlagHasEnvelope)) return rbf::LinkEnvelope{};
        StoredLinkAabb a = flat_payload_->read_link_aabb(node, ci);
        if (a.n_active_links == 0) return rbf::LinkEnvelope{};
        rbf::LinkEnvelope result;
        result.type            = rbf::EnvelopeType::LinkIAABB;
        result.n_active_links  = a.n_active_links;
        result.n_subdivisions  = a.n_subdivisions;
        result.active_link_radii.assign(a.active_link_radii.begin(), a.active_link_radii.end());
        result.link_iaabbs = decode_f16(a.link_iaabbs_f16);
        // Re-derive Scheme-A dropped fields (same logic as heap path below).
        const int n = a.n_active_links;
        result.inflated_link_iaabbs = result.link_iaabbs;
        for (int link = 0; link < n; ++link) {
            float r = (link < static_cast<int>(a.active_link_radii.size()))
                        ? static_cast<float>(a.active_link_radii[static_cast<std::size_t>(link)]) : 0.f;
            float* box = result.inflated_link_iaabbs.data() + link * 6;
            box[0]-=r; box[1]-=r; box[2]-=r; box[3]+=r; box[4]+=r; box[5]+=r;
        }
        result.link_union_iaabbs = result.inflated_link_iaabbs;  // n_sub=1, union==inflated
        result.envelope_aabb.resize(6);
        std::copy(result.inflated_link_iaabbs.begin(), result.inflated_link_iaabbs.begin()+6, result.envelope_aabb.begin());
        for (int i = 1; i < n; ++i) {
            const float* box = result.inflated_link_iaabbs.data() + i*6;
            for (int ax = 0; ax < 3; ++ax) {
                result.envelope_aabb[static_cast<std::size_t>(ax)]   = std::min(result.envelope_aabb[static_cast<std::size_t>(ax)],   box[ax]);
                result.envelope_aabb[static_cast<std::size_t>(ax+3)] = std::max(result.envelope_aabb[static_cast<std::size_t>(ax+3)], box[ax+3]);
            }
        }
        return result;
    };

    if (!nodes_[node] && node < flat_payload_cutoff_ && flat_payload_) {
        return reconstruct_flat();
    }

    if (!nodes_[node]) return rbf::LinkEnvelope{};
    const auto& slot = nodes_[node]->channels[ci];
    rbf::LinkEnvelope result;
    // Start from AABB slot (base for every envelope type).
    if (slot.link_aabb_envelope) {
        const auto& a = *slot.link_aabb_envelope;
        result.type = rbf::EnvelopeType::LinkIAABB;
        result.n_active_links = a.n_active_links;
        result.n_subdivisions = a.n_subdivisions;
        result.active_link_radii.assign(a.active_link_radii.begin(), a.active_link_radii.end());
        // Decode float16 link_iaabbs.
        result.link_iaabbs = decode_f16(a.link_iaabbs_f16);
        // Re-derive Scheme-A dropped fields.
        const int n     = a.n_active_links;
        const int n_sub = std::max(1, a.n_subdivisions);
        const int n_boxes = n * n_sub;
        // inflated_link_iaabbs = link_iaabbs ± radius
        result.inflated_link_iaabbs = result.link_iaabbs;
        for (int link = 0; link < n; ++link) {
            float r = (link < static_cast<int>(a.active_link_radii.size()))
                        ? static_cast<float>(a.active_link_radii[static_cast<std::size_t>(link)])
                        : 0.0f;
            for (int s = 0; s < n_sub; ++s) {
                float* box = result.inflated_link_iaabbs.data() + (link * n_sub + s) * 6;
                box[0] -= r; box[1] -= r; box[2] -= r;
                box[3] += r; box[4] += r; box[5] += r;
            }
        }
        // link_union_iaabbs = per-link union of inflated over subdivisions
        result.link_union_iaabbs.resize(static_cast<std::size_t>(n * 6));
        for (int link = 0; link < n; ++link) {
            float* dst = result.link_union_iaabbs.data() + link * 6;
            const float* first = result.inflated_link_iaabbs.data() + link * n_sub * 6;
            std::copy(first, first + 6, dst);
            for (int s = 1; s < n_sub; ++s) {
                const float* box = result.inflated_link_iaabbs.data() + (link * n_sub + s) * 6;
                for (int ax = 0; ax < 3; ++ax) {
                    dst[ax]     = std::min(dst[ax],     box[ax]);
                    dst[ax + 3] = std::max(dst[ax + 3], box[ax + 3]);
                }
            }
        }
        // envelope_aabb = union of all inflated boxes
        result.envelope_aabb.resize(6);
        std::copy(result.inflated_link_iaabbs.begin(),
                  result.inflated_link_iaabbs.begin() + 6,
                  result.envelope_aabb.begin());
        for (int i = 1; i < n_boxes; ++i) {
            const float* box = result.inflated_link_iaabbs.data() + i * 6;
            for (int ax = 0; ax < 3; ++ax) {
                result.envelope_aabb[static_cast<std::size_t>(ax)]     = std::min(result.envelope_aabb[static_cast<std::size_t>(ax)],     box[ax]);
                result.envelope_aabb[static_cast<std::size_t>(ax + 3)] = std::max(result.envelope_aabb[static_cast<std::size_t>(ax + 3)], box[ax + 3]);
            }
        }
    }
    // Overlay KDop if present.
    if (slot.link_kdop_envelope) {
        const auto& k = *slot.link_kdop_envelope;
        if (result.type == rbf::EnvelopeType::LinkIAABB) result.type = rbf::EnvelopeType::KDOP;
        if (!slot.link_aabb_envelope) {
            result.n_active_links = k.n_active_links;
            result.n_subdivisions = k.n_subdivisions;
        }
        result.kdop_n_axes = k.kdop_n_axes;
        result.kdop_direction_set = k.kdop_direction_set;
        result.kdop_intervals = k.kdop_intervals;
    }
    // Overlay SupportHull if present (highest-priority type tag).
    if (slot.link_sh_envelope) {
        const auto& sh = *slot.link_sh_envelope;
        result.type = rbf::EnvelopeType::SupportHull;
        if (!slot.link_aabb_envelope && !slot.link_kdop_envelope) {
            result.n_active_links = sh.n_active_links;
            result.n_subdivisions = sh.n_subdivisions;
        }
        result.support_hulls = sh.support_hulls;
    }
    if (result.n_active_links == 0 && node < flat_payload_cutoff_ && flat_payload_) {
        return reconstruct_flat();
    }
    return result;
}

std::optional<Channel> EvidenceStore::find_serving_channel(
    int node,
    const rbf::EndpointSourceConfig& requested) const
{
    require_node(node);
    const int base = node * kNumChannels;
    for (int ci = 0; ci < kNumChannels; ++ci) {
        const uint8_t f = node_channel_flags_[base + ci];
        if (!(f & kFlagHasAny)) continue;
        const auto src = flag_source(f);
        // Quality meta: fetch from heap when available.
        // For certified sources (IFK, AAFK, HIFK_AA) the default (zero) meta
        // is always sufficient since they have no quality parameters.
        rbf::SourceQualityMetadata meta{};
        if (node < static_cast<int>(nodes_.size()) && nodes_[node])
            meta = nodes_[node]->channels[ci].quality_meta;
        if (rbf::cached_source_dominates(src, meta, requested.source, requested))
            return static_cast<Channel>(ci);
    }
    return std::nullopt;
}

std::optional<Channel> EvidenceStore::find_serving_channel(
    int node,
    rbf::EndpointSource requested) const
{
    require_node(node);
    const int base = node * kNumChannels;
    for (int ci = 0; ci < kNumChannels; ++ci) {
        const uint8_t f = node_channel_flags_[base + ci];
        if (!(f & kFlagHasAny)) continue;
        if (rbf::source_can_serve(flag_source(f), requested))
            return static_cast<Channel>(ci);
    }
    return std::nullopt;
}

std::vector<float> EvidenceStore::derive_link_aabbs(
    int node, Channel channel, const std::vector<double>& link_radii) const {
    const auto& ep = endpoint(node, channel);
    if (!link_radii.empty() && static_cast<int>(link_radii.size()) != layout_.n_active_links) {
        throw std::invalid_argument("LECT link radii length does not match active link count");
    }

    std::vector<float> link(static_cast<std::size_t>(layout_.link_stride()), 0.0f);
    for (int link_idx = 0; link_idx < layout_.n_active_links; ++link_idx) {
        const float radius = link_radii.empty()
            ? 0.0f
            : static_cast<float>(link_radii[static_cast<std::size_t>(link_idx)]);
        const int prox = (link_idx * 2) * 6;
        const int dist = (link_idx * 2 + 1) * 6;
        const int out = link_idx * 6;
        for (int axis = 0; axis < 3; ++axis) {
            link[out + axis] = std::min(ep[prox + axis], ep[dist + axis]) - radius;
            link[out + axis + 3] = std::max(ep[prox + axis + 3], ep[dist + axis + 3]) + radius;
        }
    }
    return link;
}

std::vector<float> EvidenceStore::get_or_derive_link_aabbs(
    int node, Channel channel, const std::vector<double>& link_radii) const {
    if (link_radii.empty() && has_link_aabbs(node, channel)) {
        auto stored = link_aabbs(node, channel);
        if (static_cast<int>(stored.size()) == layout_.link_stride()) {
            return stored;
        }
    }
    return derive_link_aabbs(node, channel, link_radii);
}

EvidenceStore::ParentRefinementResult EvidenceStore::refine_parent_from_children(
    int parent, int left_child, int right_child, Channel channel, rbf::EndpointSource source,
    bool store_link_aabb) {
    require_node(parent);
    require_node(left_child);
    require_node(right_child);
    require_channel(channel);
    if (!has_endpoint(left_child, channel) || !has_endpoint(right_child, channel)) {
        return {false, false, channel};
    }
    const auto left_endpoint  = endpoint(left_child, channel);
    const auto right_endpoint = endpoint(right_child, channel);
    auto refined_endpoint = union_aabbs(left_endpoint, right_endpoint, layout_.endpoint_stride());
    set_endpoint(parent, channel, source, std::move(refined_endpoint));

    ParentRefinementResult result;
    result.refined_endpoint = true;
    result.channel = channel;
    if (store_link_aabb) {
        const auto left_link = get_or_derive_link_aabbs(left_child, channel);
        const auto right_link = get_or_derive_link_aabbs(right_child, channel);
        set_link_aabbs(parent, channel, union_aabbs(left_link, right_link, layout_.link_stride()));
        result.refined_link_aabb = true;
    }
    return result;
}

void EvidenceStore::require_node(int node) const {
    if (node < 0 || node >= n_nodes()) {
        throw std::out_of_range("LECT evidence node index is out of range");
    }
}

void EvidenceStore::require_channel(Channel channel) {
    const int index = channel_index(channel);
    if (index < 0 || index >= kNumChannels) {
        throw std::out_of_range("LECT evidence channel is out of range");
    }
}

void EvidenceStore::require_endpoint_payload(const std::vector<float>& endpoint_iaabbs) const {
    if (static_cast<int>(endpoint_iaabbs.size()) != layout_.endpoint_stride()) {
        throw std::invalid_argument("LECT endpoint evidence has unexpected length");
    }
}

void EvidenceStore::require_link_payload(const std::vector<float>& link_aabbs) const {
    if (static_cast<int>(link_aabbs.size()) != layout_.link_stride()) {
        throw std::invalid_argument("LECT link AABB evidence has unexpected length");
    }
}

void EvidenceStore::hydrate_flat_payload_flags(int first_node, int last_node) {
    if (!flat_payload_ || !flat_payload_->valid()) {
        return;
    }
    const int begin = std::max(0, first_node);
    const int end = std::min({last_node,
                              n_nodes(),
                              static_cast<int>(std::min<std::uint64_t>(
                                  flat_payload_->n_nodes,
                                  static_cast<std::uint64_t>(std::numeric_limits<int>::max())))});
    if (begin >= end) {
        return;
    }
    for (int node = begin; node < end; ++node) {
        for (int ci = 0; ci < kNumChannels; ++ci) {
            const uint8_t record_flags = flat_payload_->record_ptr(node, ci)[0];
            if (!(record_flags & kFlagHasAny)) {
                continue;
            }
            uint8_t& hot_flags = node_channel_flags_[node * kNumChannels + ci];
            if (hot_flags & kFlagHasAny) {
                continue;
            }
            hot_flags = record_flags & (kFlagHasAny | kFlagSrcMask);
        }
    }
}


std::vector<float> EvidenceStore::union_aabbs(const std::vector<float>& lhs,
                                              const std::vector<float>& rhs,
                                              int stride) {
    if (static_cast<int>(lhs.size()) != stride || static_cast<int>(rhs.size()) != stride || stride % 6 != 0) {
        throw std::invalid_argument("LECT AABB payloads cannot be unioned");
    }
    std::vector<float> out(static_cast<std::size_t>(stride));
    for (int box = 0; box < stride / 6; ++box) {
        const int offset = box * 6;
        for (int axis = 0; axis < 3; ++axis) {
            out[offset + axis] = std::min(lhs[offset + axis], rhs[offset + axis]);
            out[offset + axis + 3] = std::max(lhs[offset + axis + 3], rhs[offset + axis + 3]);
        }
    }
    return out;
}

// ─── Disk-backed flat payload ─────────────────────────────────────────────────

void EvidenceStore::save_flat_payload(const std::filesystem::path& path) const {
    const int N  = n_nodes();
    const int nA = layout_.n_active_links;
    const int ep_f16_count   = nA * 2 * 6;  // float16 elements for endpoint
    const uint32_t stride      = static_cast<uint32_t>(4 + ep_f16_count * 2);
    const uint32_t radii_bytes = static_cast<uint32_t>(nA * 4);
    const uint32_t data_offset = 32u + radii_bytes;

    FlatPayloadFileHeader hdr{};
    hdr.magic         = kFlatPayloadMagic;
    hdr.version       = kFlatPayloadVersion;
    hdr.n_active_links = static_cast<uint32_t>(nA);
    hdr.n_channels    = static_cast<uint32_t>(kNumChannels);
    hdr.n_nodes       = static_cast<uint64_t>(N);
    hdr.stride        = stride;
    hdr.data_offset   = data_offset;

    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("LECT flat payload: cannot open file for write: " + path.string());

    f.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    // active_link_radii: collect from first available node; robot-constant across all nodes.
    std::vector<float> radii(static_cast<std::size_t>(nA), 0.f);
    for (int node = 0; node < N && radii[0] == 0.f; ++node) {
        for (int ci = 0; ci < kNumChannels; ++ci) {
            if (!(node_channel_flags_[node * kNumChannels + ci] & kFlagHasEnvelope)) continue;
            if (node < static_cast<int>(nodes_.size()) && nodes_[node]) {
                const auto& slot = nodes_[node]->channels[ci];
                if (slot.link_aabb_envelope && !slot.link_aabb_envelope->active_link_radii.empty()) {
                    for (int i = 0; i < nA && i < static_cast<int>(slot.link_aabb_envelope->active_link_radii.size()); ++i)
                        radii[static_cast<std::size_t>(i)] = static_cast<float>(slot.link_aabb_envelope->active_link_radii[static_cast<std::size_t>(i)]);
                    break;
                }
            }
        }
    }
    f.write(reinterpret_cast<const char*>(radii.data()), nA * 4);

    // Records: one per (node, channel), fixed stride.
    std::vector<uint8_t> rec(stride, 0u);
    for (int node = 0; node < N; ++node) {
        for (int ci = 0; ci < kNumChannels; ++ci) {
            std::fill(rec.begin(), rec.end(), 0u);
            const uint8_t flags = node_channel_flags_[node * kNumChannels + ci];
            if (flags & kFlagHasAny) {
                uint8_t rec_flags = 0u;
                bool wrote_heap_record = false;
                if (node < static_cast<int>(nodes_.size()) && nodes_[node]) {
                    const auto& slot = nodes_[node]->channels[ci];
                    wrote_heap_record = slot.has || slot.link_aabb_envelope.has_value();
                    if (slot.has && !slot.endpoint_f16.empty()) {
                        rec_flags = make_flag(true, false, flag_source(flags));
                        const auto* src = slot.endpoint_f16.data();
                        std::memcpy(rec.data() + 4, src, static_cast<std::size_t>(ep_f16_count) * 2);
                    }
                }
                if (!wrote_heap_record && node < flat_payload_cutoff_ && flat_payload_ && flat_payload_->valid()) {
                    // Previously-spilled: copy raw record bytes directly (same stride/format).
                    const uint8_t* src = flat_payload_->record_ptr(node, ci);
                    std::memcpy(rec.data(), src, stride);
                } else {
                    rec[0] = rec_flags;
                }
            }
            f.write(reinterpret_cast<const char*>(rec.data()), stride);
        }
    }
    if (!f) throw std::runtime_error("LECT flat payload: write failed: " + path.string());
}

void EvidenceStore::attach_flat_payload(const std::filesystem::path& path) {
    auto fp = std::make_unique<FlatPayloadFile>();
    fp->path = path;
    fp->mapped = MappedLectFile::open_read_only(path, /*random_access_advice=*/true);
    if (!fp->mapped.is_open())
        throw std::runtime_error("LECT flat payload: cannot mmap file: " + path.string());

    if (fp->mapped.size() < sizeof(FlatPayloadFileHeader))
        throw std::runtime_error("LECT flat payload: file is too small: " + path.string());

    const auto* hdr = reinterpret_cast<const FlatPayloadFileHeader*>(fp->mapped.bytes().data());
    if (hdr->magic != kFlatPayloadMagic)
        throw std::runtime_error("LECT flat payload: bad magic number");
    if (hdr->version != kFlatPayloadVersion)
        throw std::runtime_error("LECT flat payload: unsupported version");
    if (static_cast<int>(hdr->n_active_links) != layout_.n_active_links)
        throw std::runtime_error("LECT flat payload: n_active_links mismatch");
    if (hdr->n_channels != kNumChannels)
        throw std::runtime_error("LECT flat payload: channel count mismatch");
    if (hdr->n_nodes > static_cast<uint64_t>(std::numeric_limits<int>::max()))
        throw std::runtime_error("LECT flat payload: node count overflow");
    const std::size_t min_stride = 4u + static_cast<std::size_t>(hdr->n_active_links) * 2u * 6u * sizeof(uint16_t);
    const std::size_t min_data_offset = sizeof(FlatPayloadFileHeader) +
        static_cast<std::size_t>(hdr->n_active_links) * sizeof(float);
    if (hdr->stride < min_stride)
        throw std::runtime_error("LECT flat payload: invalid record stride");
    if (hdr->data_offset < min_data_offset || hdr->data_offset > fp->mapped.size())
        throw std::runtime_error("LECT flat payload: invalid data offset");
    const std::size_t records = static_cast<std::size_t>(hdr->n_nodes) *
        static_cast<std::size_t>(hdr->n_channels);
    if (records != 0 && hdr->stride >
            (std::numeric_limits<std::size_t>::max() - hdr->data_offset) / records) {
        throw std::runtime_error("LECT flat payload: file size overflow");
    }
    const std::size_t expected_size = static_cast<std::size_t>(hdr->data_offset) +
        records * static_cast<std::size_t>(hdr->stride);
    if (expected_size > fp->mapped.size())
        throw std::runtime_error("LECT flat payload: truncated file");
    // hdr->n_nodes may exceed the current tree size (e.g. the payload was saved
    // after a prewarm but the session starts with a fresh 1-node tree).
    // discard_heap_payloads() caps flat_payload_cutoff_ to n_nodes(), so it is
    // safe to accept a payload that covers more nodes than currently exist.

    fp->n_active_links = hdr->n_active_links;
    fp->n_nodes        = hdr->n_nodes;
    fp->stride         = hdr->stride;
    fp->data_offset    = hdr->data_offset;

    // Decode active_link_radii from file.
    const int nA = static_cast<int>(hdr->n_active_links);
    const auto* radii_ptr = reinterpret_cast<const float*>(fp->mapped.bytes().data() + 32);
    fp->active_link_radii.assign(radii_ptr, radii_ptr + nA);

    flat_payload_ = std::move(fp);
    flat_payload_cutoff_ = static_cast<int>(hdr->n_nodes);
    hydrate_flat_payload_flags(0, static_cast<int>(std::min<std::uint64_t>(
        hdr->n_nodes, static_cast<std::uint64_t>(n_nodes()))));
}

void EvidenceStore::discard_heap_payloads() {
    if (flat_payload_) {
        flat_payload_cutoff_ = static_cast<int>(std::min<std::uint64_t>(
            flat_payload_->n_nodes, static_cast<std::uint64_t>(n_nodes())));
    } else {
        flat_payload_cutoff_ = n_nodes();
    }
    for (int i = 0; i < flat_payload_cutoff_; ++i) {
        nodes_[static_cast<std::size_t>(i)].reset();
    }
}

void EvidenceStore::spill_to_disk(const std::filesystem::path& path) {
    // Write everything (heap evidence + any previously-spilled flat_payload_ data)
    // to a temporary file, then atomically replace the target path.
    const auto parent = path.parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    const auto tmp = std::filesystem::path(path.string() + ".spill_tmp");
    save_flat_payload(tmp);      // merges heap + old flat_payload_ (now fixed)
    flat_payload_.reset();       // release old mmap before rename
    flat_payload_cutoff_ = 0;    // reset: new file covers everything
    std::filesystem::rename(tmp, path);
    attach_flat_payload(path);
    discard_heap_payloads();     // sets flat_payload_cutoff_ = n_nodes(), frees heap
}

bool EvidenceStore::has_flat_payload() const {
    return flat_payload_ != nullptr && flat_payload_->valid();
}

bool EvidenceStore::flat_payload_covers_all_nodes() const {
    return has_flat_payload() && flat_payload_cutoff_ >= n_nodes();
}

}  // namespace lect