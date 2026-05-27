#include <rbf/lect_database/types.h>

#include <functional>
#include <iomanip>
#include <sstream>

namespace rbf::lect_database {

bool valid_node_id(NodeId id) noexcept {
    return id != kInvalidNodeId;
}

std::uint64_t stable_hash_append(std::uint64_t hash, const void* data, std::size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (std::size_t index = 0; index < size; ++index) {
        hash ^= static_cast<std::uint64_t>(bytes[index]);
        hash *= 1099511628211ull;
    }
    return hash;
}

std::uint64_t stable_hash_append(std::uint64_t hash, std::string_view text) {
    for (char ch : text) {
        hash ^= static_cast<std::uint64_t>(static_cast<unsigned char>(ch));
        hash *= 1099511628211ull;
    }
    return hash;
}

std::uint64_t stable_hash(std::string_view text) {
    return stable_hash_append(1469598103934665603ull, text);
}

std::uint64_t fingerprint_intervals(const std::vector<Interval>& intervals) {
    std::uint64_t hash = 1469598103934665603ull;
    for (const auto& interval : intervals) {
        hash = stable_hash_append(hash, &interval.lo, sizeof(interval.lo));
        hash = stable_hash_append(hash, &interval.hi, sizeof(interval.hi));
    }
    return hash;
}

std::string interval_descriptor(const std::vector<Interval>& intervals) {
    std::ostringstream out;
    out << "dims=" << intervals.size();
    for (const auto& interval : intervals) {
        out << '|'
            << std::setprecision(17) << interval.lo
            << ':'
            << std::setprecision(17) << interval.hi;
    }
    return out.str();
}

void PathCode::push_child(bool right_child) {
    const int word_index = bit_count / 64;
    const int bit_index = bit_count % 64;
    if (word_index >= static_cast<int>(words.size())) {
        words.push_back(0);
    }
    if (right_child) {
        words[static_cast<std::size_t>(word_index)] |= (std::uint64_t{1} << bit_index);
    }
    ++bit_count;
}

bool PathCode::bit(int index) const {
    if (index < 0 || index >= bit_count) {
        return false;
    }
    const int word_index = index / 64;
    const int bit_index = index % 64;
    return (words[static_cast<std::size_t>(word_index)] & (std::uint64_t{1} << bit_index)) != 0;
}

bool PathCode::is_prefix_of(const PathCode& other) const {
    if (bit_count > other.bit_count) {
        return false;
    }
    return common_prefix_bits(other) == bit_count;
}

int PathCode::common_prefix_bits(const PathCode& other) const {
    const int limit = std::min(bit_count, other.bit_count);
    for (int index = 0; index < limit; ++index) {
        if (bit(index) != other.bit(index)) {
            return index;
        }
    }
    return limit;
}

bool PathCode::operator==(const PathCode& other) const noexcept {
    return bit_count == other.bit_count && words == other.words;
}

std::size_t PathCodeHash::operator()(const PathCode& path) const noexcept {
    std::size_t seed = 0;
    auto mix = [&](std::uint64_t value) {
        seed ^= std::hash<std::uint64_t>{}(value) + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
    };
    mix(static_cast<std::uint64_t>(path.bit_count));
    for (std::uint64_t word : path.words) {
        mix(word);
    }
    return seed;
}

bool EvidenceKey::operator==(const EvidenceKey& other) const noexcept {
    const bool same_node = node_path_valid && other.node_path_valid
        ? node_path == other.node_path
        : node_id == other.node_id;
    return same_node &&
           sector == other.sector &&
           channel == other.channel &&
           endpoint_source == other.endpoint_source &&
           payload_kind == other.payload_kind;
}

std::size_t EvidenceKeyHash::operator()(const EvidenceKey& key) const noexcept {
    std::size_t seed = 0;
    auto mix = [&](auto value) {
        seed ^= std::hash<decltype(value)>{}(value) + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
    };
    mix(static_cast<int>(key.node_path_valid));
    if (key.node_path_valid) {
        mix(PathCodeHash{}(key.node_path));
    } else {
        mix(key.node_id);
    }
    mix(key.sector);
    mix(static_cast<int>(key.channel));
    mix(static_cast<int>(key.endpoint_source));
    mix(static_cast<int>(key.payload_kind));
    return seed;
}

}  // namespace rbf::lect_database
