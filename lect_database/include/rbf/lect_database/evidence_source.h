#pragma once

#include <rbf/lect_database/database.h>
#include <rbf/lect_database/read_snapshot.h>

#include <optional>
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
        return snapshot_->endpoint_for_box_exact(snapshot_->make_box_key(intervals), key_template);
    }

private:
    const LectReadSnapshot* snapshot_ = nullptr;
};

}  // namespace rbf::lect_database