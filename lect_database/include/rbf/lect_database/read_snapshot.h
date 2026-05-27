#pragma once

#include <rbf/lect_database/identity.h>
#include <rbf/lect_database/split_policy.h>
#include <rbf/lect_database/types.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rbf::lect_database {

class LectReadSnapshot {
public:
    LectReadSnapshot();
    ~LectReadSnapshot();
    LectReadSnapshot(LectReadSnapshot&&) noexcept;
    LectReadSnapshot& operator=(LectReadSnapshot&&) noexcept;
    LectReadSnapshot(const LectReadSnapshot&) = delete;
    LectReadSnapshot& operator=(const LectReadSnapshot&) = delete;

    static std::filesystem::path default_snapshot_path(const std::filesystem::path& legacy_root);
    static bool build_from_legacy(const std::filesystem::path& legacy_root,
                                  const std::filesystem::path& snapshot_path,
                                  std::string* reason = nullptr);

    bool open(const std::filesystem::path& snapshot_path, std::string* reason = nullptr);
    void close();
    bool is_open() const noexcept;

    std::size_t node_count() const noexcept;
    std::size_t evidence_count() const noexcept;
    std::uint64_t generation() const noexcept;
    const std::vector<Interval>& root_intervals() const noexcept;

    BoxKey make_box_key(std::vector<Interval> intervals) const;
    std::optional<std::vector<Interval>> node_box(NodeId node_id) const;
    BoxLookupResult box_to_node_exact(const BoxKey& box) const;
    std::vector<NodeId> range_query(const std::vector<Interval>& box,
                                    RangeQueryMode mode,
                                    LectDatabaseStats* stats = nullptr) const;
    std::optional<EvidenceRecordView> evidence(const EvidenceKey& key) const;
    std::optional<EvidenceRecordView> endpoint_for_box_exact(const BoxKey& box,
                                                             EvidenceKey key_template) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rbf::lect_database
