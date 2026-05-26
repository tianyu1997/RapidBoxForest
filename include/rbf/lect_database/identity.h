#pragma once

#include <rbf/core/robot.h>
#include <rbf/lect_database/split_policy.h>

#include <string>

namespace rbf::lect_database {

struct LectDatabaseIdentity {
    std::uint32_t schema_version = kLectDatabaseSchemaVersion;
    std::uint64_t robot_fingerprint = 0;
    std::uint64_t root_domain_fingerprint = 0;
    std::uint64_t split_policy_hash = 0;
    bool canonical_mode = false;
    std::string split_policy_descriptor;
    std::string endpoint_descriptor = "endpoint_identity_unspecified";
    std::string envelope_descriptor = "envelope_identity_unspecified";
    std::string payload_layout = "endpoint_envelope_v1";
    std::string builder_version;
};

std::string identity_descriptor(const LectDatabaseIdentity& identity);
std::uint64_t identity_hash(const LectDatabaseIdentity& identity);
bool identity_compatible(const LectDatabaseIdentity& stored,
                         const LectDatabaseIdentity& requested,
                         std::string* reason = nullptr);
LectDatabaseIdentity make_identity_for_robot(const Robot& robot,
                                             const std::vector<Interval>& root_intervals,
                                             const SplitPolicyDescriptor& split_policy,
                                             bool canonical_mode = false,
                                             std::string endpoint_descriptor = {},
                                             std::string envelope_descriptor = {},
                                             std::string payload_layout = {},
                                             std::string builder_version = {});

}  // namespace rbf::lect_database
