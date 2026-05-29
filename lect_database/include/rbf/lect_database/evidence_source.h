#pragma once

#include <rbf/lect_database/database.h>
#include <rbf/lect_database/read_snapshot.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

namespace rbf::lect_database {

class LectExternalEvidenceSource {
public:
    virtual ~LectExternalEvidenceSource() = default;

    virtual std::optional<EvidenceRecordView> evidence(const EvidenceKey& key) const = 0;
    virtual std::optional<EvidenceRecordView> endpoint_for_box_exact(const std::vector<Interval>& intervals,
                                                                     EvidenceKey key_template) const = 0;
};

class LectDatabaseEvidenceSource final : public LectExternalEvidenceSource {
public:
    LectDatabaseEvidenceSource() = default;
    explicit LectDatabaseEvidenceSource(const LectDatabase& database) : database_(&database) {}

    void reset(const LectDatabase& database) { database_ = &database; }
    void clear() noexcept { database_ = nullptr; }

    std::optional<EvidenceRecordView> evidence(const EvidenceKey& key) const override {
        if (database_ == nullptr) {
            return std::nullopt;
        }
        return database_->evidence(key);
    }

    std::optional<EvidenceRecordView> endpoint_for_box_exact(const std::vector<Interval>& intervals,
                                                             EvidenceKey key_template) const override {
        if (database_ == nullptr) {
            return std::nullopt;
        }
        return database_->endpoint_for_box_exact(database_->make_box_key(intervals), key_template);
    }

private:
    const LectDatabase* database_ = nullptr;
};

class LectSnapshotEvidenceSource final : public LectExternalEvidenceSource {
public:
    LectSnapshotEvidenceSource() = default;
    explicit LectSnapshotEvidenceSource(const LectReadSnapshot& snapshot) : snapshot_(&snapshot) {}

    void reset(const LectReadSnapshot& snapshot) { snapshot_ = &snapshot; }
    void clear() noexcept { snapshot_ = nullptr; }

    std::optional<EvidenceRecordView> evidence(const EvidenceKey& key) const override {
        if (snapshot_ == nullptr) {
            return std::nullopt;
        }
        return snapshot_->evidence(key);
    }

    std::optional<EvidenceRecordView> endpoint_for_box_exact(const std::vector<Interval>& intervals,
                                                             EvidenceKey key_template) const override {
        if (snapshot_ == nullptr) {
            return std::nullopt;
        }
        return snapshot_->endpoint_for_box_exact(intervals, key_template);
    }

private:
    const LectReadSnapshot* snapshot_ = nullptr;
};

/// Thread-safe, interval-keyed, in-memory endpoint evidence cache shared by all
/// worker oracles spawned from a single master. It lets concurrent build tasks
/// reuse endpoints computed by sibling tasks (and by earlier batches), the way a
/// single persistent oracle reuses evidence across queries. Lookups verify an
/// exact (bit-for-bit) interval match, so a returned record always corresponds
/// to the queried box; fingerprint collisions never produce a wrong payload.
class SharedEndpointEvidenceCache final : public LectExternalEvidenceSource {
public:
    SharedEndpointEvidenceCache() = default;
    SharedEndpointEvidenceCache(std::size_t max_entries, std::size_t max_bytes)
        : max_entries_(max_entries), max_bytes_(max_bytes) {}

    // 0 means "unbounded" for the corresponding dimension.
    void set_limits(std::size_t max_entries, std::size_t max_bytes) {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        max_entries_ = max_entries;
        max_bytes_ = max_bytes;
        evict_if_needed_locked();
    }

    std::optional<EvidenceRecordView> evidence(const EvidenceKey&) const override {
        // Node-id keys are worker-local and meaningless across tasks; this cache
        // is exclusively interval-keyed.
        return std::nullopt;
    }

    std::optional<EvidenceRecordView> endpoint_for_box_exact(const std::vector<Interval>& intervals,
                                                             EvidenceKey key_template) const override {
        const std::uint64_t fp = fingerprint_intervals(intervals);
        std::shared_lock<std::shared_mutex> lock(mutex_);
        const auto bucket = map_.find(fp);
        if (bucket == map_.end()) {
            return std::nullopt;
        }
        for (const auto& entry : bucket->second) {
            if (!intervals_equal(entry.intervals, intervals)) {
                continue;
            }
            const auto& record = *entry.record;
            // Only reuse when the payload semantics match the lookup intent.
            if (record.key.channel != key_template.channel ||
                record.key.endpoint_source != key_template.endpoint_source ||
                record.key.payload_kind != key_template.payload_kind) {
                continue;
            }
            // Touch the LRU stamp without upgrading to a write lock; the atomic
            // store is safe under the shared lock (no structural mutation).
            entry.last_used_seq.store(tick_.fetch_add(1, std::memory_order_relaxed) + 1,
                                      std::memory_order_relaxed);
            EvidenceRecordView view;
            view.key = key_template;
            view.child_hull = record.child_hull;
            view.unavailable = record.unavailable;
            view.payload = record.payload;
            view.storage = entry.record;  // keeps payload alive for the reader
            return view;
        }
        return std::nullopt;
    }

    void put(const std::vector<Interval>& intervals,
             EvidenceKey key,
             std::vector<float> payload,
             bool child_hull,
             bool unavailable) {
        auto record = std::make_shared<EvidenceRecord>();
        record->key = key;
        record->child_hull = child_hull;
        record->unavailable = unavailable;
        record->payload = std::move(payload);
        const std::size_t new_bytes = entry_bytes(intervals, *record);
        const std::uint64_t fp = fingerprint_intervals(intervals);
        const std::uint64_t seq = tick_.fetch_add(1, std::memory_order_relaxed) + 1;
        std::unique_lock<std::shared_mutex> lock(mutex_);
        auto& bucket = map_[fp];
        for (auto& entry : bucket) {
            if (intervals_equal(entry.intervals, intervals) &&
                entry.record->key.channel == key.channel &&
                entry.record->key.endpoint_source == key.endpoint_source &&
                entry.record->key.payload_kind == key.payload_kind) {
                const std::size_t old_bytes = entry.bytes;
                entry.record = std::move(record);
                entry.bytes = new_bytes;
                entry.last_used_seq.store(seq, std::memory_order_relaxed);
                current_bytes_ = current_bytes_ - old_bytes + new_bytes;
                evict_if_needed_locked();
                return;
            }
        }
        bucket.emplace_back();
        Entry& entry = bucket.back();
        entry.intervals = intervals;
        entry.record = std::move(record);
        entry.bytes = new_bytes;
        entry.last_used_seq.store(seq, std::memory_order_relaxed);
        current_entries_ += 1;
        current_bytes_ += new_bytes;
        evict_if_needed_locked();
    }

    void clear() {
        std::unique_lock<std::shared_mutex> lock(mutex_);
        map_.clear();
        current_entries_ = 0;
        current_bytes_ = 0;
    }

    std::size_t size() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return current_entries_;
    }

    std::size_t bytes() const {
        std::shared_lock<std::shared_mutex> lock(mutex_);
        return current_bytes_;
    }

    std::uint64_t evictions() const {
        return evictions_.load(std::memory_order_relaxed);
    }

private:
    struct Entry {
        std::vector<Interval> intervals;
        std::shared_ptr<const EvidenceRecord> record;
        std::size_t bytes = 0;
        mutable std::atomic<std::uint64_t> last_used_seq{0};

        Entry() = default;
        Entry(const Entry& other)
            : intervals(other.intervals),
              record(other.record),
              bytes(other.bytes),
              last_used_seq(other.last_used_seq.load(std::memory_order_relaxed)) {}
        Entry& operator=(const Entry& other) {
            if (this != &other) {
                intervals = other.intervals;
                record = other.record;
                bytes = other.bytes;
                last_used_seq.store(other.last_used_seq.load(std::memory_order_relaxed),
                                    std::memory_order_relaxed);
            }
            return *this;
        }
    };

    static bool intervals_equal(const std::vector<Interval>& a, const std::vector<Interval>& b) {
        if (a.size() != b.size()) {
            return false;
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (a[i].lo != b[i].lo || a[i].hi != b[i].hi) {
                return false;
            }
        }
        return true;
    }

    static std::size_t entry_bytes(const std::vector<Interval>& intervals, const EvidenceRecord& record) {
        return sizeof(Entry) + intervals.size() * sizeof(Interval) +
               record.payload.size() * sizeof(float);
    }

    // Caller must hold the unique (write) lock.
    void evict_if_needed_locked() {
        const bool entries_over = max_entries_ != 0 && current_entries_ > max_entries_;
        const bool bytes_over = max_bytes_ != 0 && current_bytes_ > max_bytes_;
        if (!entries_over && !bytes_over) {
            return;
        }
        // Repeatedly drop the globally least-recently-used entry until back in
        // budget. Buckets are small, so a linear scan for the min is acceptable.
        while ((max_entries_ != 0 && current_entries_ > max_entries_) ||
               (max_bytes_ != 0 && current_bytes_ > max_bytes_)) {
            std::uint64_t best_seq = std::numeric_limits<std::uint64_t>::max();
            std::uint64_t best_fp = 0;
            std::size_t best_idx = 0;
            bool found = false;
            for (auto& [fp, bucket] : map_) {
                for (std::size_t i = 0; i < bucket.size(); ++i) {
                    const std::uint64_t seq = bucket[i].last_used_seq.load(std::memory_order_relaxed);
                    if (!found || seq < best_seq) {
                        best_seq = seq;
                        best_fp = fp;
                        best_idx = i;
                        found = true;
                    }
                }
            }
            if (!found) {
                break;
            }
            auto bucket_it = map_.find(best_fp);
            if (bucket_it == map_.end() || bucket_it->second.empty()) {
                break;
            }
            auto& bucket = bucket_it->second;
            current_bytes_ -= bucket[best_idx].bytes;
            current_entries_ -= 1;
            bucket.erase(bucket.begin() + static_cast<std::ptrdiff_t>(best_idx));
            if (bucket.empty()) {
                map_.erase(bucket_it);
            }
            evictions_.fetch_add(1, std::memory_order_relaxed);
        }
    }

    mutable std::shared_mutex mutex_;
    std::unordered_map<std::uint64_t, std::vector<Entry>> map_;
    std::size_t max_entries_ = 0;
    std::size_t max_bytes_ = 0;
    std::size_t current_entries_ = 0;
    std::size_t current_bytes_ = 0;
    mutable std::atomic<std::uint64_t> tick_{0};
    std::atomic<std::uint64_t> evictions_{0};
};

}  // namespace rbf::lect_database