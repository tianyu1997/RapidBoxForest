#include <rbf/lect_database/database.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace rbf::lect_database {

namespace {

constexpr std::size_t kEvidenceIndexLoadFactorNumeratorLocal = 13;
constexpr std::size_t kEvidenceIndexLoadFactorDenominatorLocal = 16;

std::size_t next_power_of_two_local(std::size_t value) {
    std::size_t result = 1;
    while (result < value) {
        result <<= 1;
    }
    return result;
}

std::size_t evidence_index_slot_count_local(std::size_t item_count) {
    if (item_count == 0) {
        return 16;
    }
    const auto required_slots = (item_count * kEvidenceIndexLoadFactorDenominatorLocal +
                                 kEvidenceIndexLoadFactorNumeratorLocal - 1) /
        kEvidenceIndexLoadFactorNumeratorLocal;
    return next_power_of_two_local(std::max<std::size_t>(16, required_slots));
}

}  // namespace

void LectDatabase::clear_evidence_index() noexcept {
    evidence_index_.clear();
    evidence_index_count_ = 0;
}

void LectDatabase::reserve_evidence_index(std::size_t item_count) {
    const auto min_slots = evidence_index_slot_count_local(item_count);
    if (evidence_index_.size() >= min_slots) {
        return;
    }

    std::vector<EvidenceIndexRecord> grown(min_slots);
    for (auto& slot : grown) {
        slot.key.node_id = kInvalidNodeId;
    }

    const auto mask = min_slots - 1;
    for (const auto& slot : evidence_index_) {
        if (slot.key.node_id == kInvalidNodeId) {
            continue;
        }
        std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(slot.key)) & mask;
        while (grown[probe].key.node_id != kInvalidNodeId) {
            probe = (probe + 1) & mask;
        }
        grown[probe] = slot;
    }
    evidence_index_ = std::move(grown);
}

const LectDatabase::EvidenceIndexEntry* LectDatabase::find_evidence_index(const EvidenceKey& key) const {
    if (evidence_index_count_ == 0 || evidence_index_.empty()) {
        return nullptr;
    }
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return nullptr;
    }
    const auto mask = evidence_index_.size() - 1;
    std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(normalized_key)) & mask;
    while (true) {
        const auto& slot = evidence_index_[probe];
        if (slot.key.node_id == kInvalidNodeId) {
            return nullptr;
        }
        if (slot.key == normalized_key) {
            return &slot.entry;
        }
        probe = (probe + 1) & mask;
    }
}

LectDatabase::EvidenceIndexEntry* LectDatabase::find_evidence_index(const EvidenceKey& key) {
    if (evidence_index_count_ == 0 || evidence_index_.empty()) {
        return nullptr;
    }
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        return nullptr;
    }
    const auto mask = evidence_index_.size() - 1;
    std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(normalized_key)) & mask;
    while (true) {
        auto& slot = evidence_index_[probe];
        if (slot.key.node_id == kInvalidNodeId) {
            return nullptr;
        }
        if (slot.key == normalized_key) {
            return &slot.entry;
        }
        probe = (probe + 1) & mask;
    }
}

void LectDatabase::upsert_evidence_index(const EvidenceKey& key, EvidenceIndexEntry entry) {
    EvidenceKey normalized_key = key;
    if (!normalize_evidence_key(&normalized_key)) {
        normalized_key = key;
    }
    if (evidence_index_.empty() ||
        evidence_index_slot_count_local(evidence_index_count_ + 1) > evidence_index_.size()) {
        reserve_evidence_index(evidence_index_count_ + 1);
    }

    const auto mask = evidence_index_.size() - 1;
    std::size_t probe = static_cast<std::size_t>(EvidenceKeyHash{}(normalized_key)) & mask;
    while (true) {
        auto& slot = evidence_index_[probe];
        if (slot.key.node_id == kInvalidNodeId) {
            slot.key = normalized_key;
            slot.entry = entry;
            ++evidence_index_count_;
            return;
        }
        if (slot.key == normalized_key) {
            slot.entry = entry;
            return;
        }
        probe = (probe + 1) & mask;
    }
}

}  // namespace rbf::lect_database
