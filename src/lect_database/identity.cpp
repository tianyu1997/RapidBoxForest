#include <rbf/lect_database/identity.h>

#include <sstream>

namespace rbf::lect_database {

std::string identity_descriptor(const LectDatabaseIdentity& identity) {
    std::ostringstream out;
    out << "schema=" << identity.schema_version
        << "|robot=" << identity.robot_fingerprint
        << "|root=" << identity.root_domain_fingerprint
        << "|canonical=" << (identity.canonical_mode ? 1 : 0)
        << "|split_hash=" << identity.split_policy_hash
        << "|split={" << identity.split_policy_descriptor << "}"
        << "|endpoint=" << identity.endpoint_descriptor
        << "|envelope=" << identity.envelope_descriptor
        << "|payload=" << identity.payload_layout
        << "|builder=" << identity.builder_version;
    return out.str();
}

std::uint64_t identity_hash(const LectDatabaseIdentity& identity) {
    return stable_hash(identity_descriptor(identity));
}

bool identity_compatible(const LectDatabaseIdentity& stored,
                         const LectDatabaseIdentity& requested,
                         std::string* reason) {
    if (stored.schema_version != requested.schema_version) {
        if (reason) *reason = "schema version differs";
        return false;
    }
    if (stored.robot_fingerprint != requested.robot_fingerprint) {
        if (reason) *reason = "robot fingerprint differs";
        return false;
    }
    if (stored.root_domain_fingerprint != requested.root_domain_fingerprint) {
        if (reason) *reason = "root domain fingerprint differs";
        return false;
    }
    if (stored.canonical_mode != requested.canonical_mode) {
        if (reason) *reason = "canonical mode differs";
        return false;
    }
    if (stored.split_policy_hash != requested.split_policy_hash ||
        stored.split_policy_descriptor != requested.split_policy_descriptor) {
        if (reason) *reason = "split policy differs";
        return false;
    }
    if (stored.endpoint_descriptor != requested.endpoint_descriptor) {
        if (reason) *reason = "endpoint descriptor differs";
        return false;
    }
    if (stored.envelope_descriptor != requested.envelope_descriptor) {
        if (reason) *reason = "envelope descriptor differs";
        return false;
    }
    if (stored.payload_layout != requested.payload_layout) {
        if (reason) *reason = "payload layout differs";
        return false;
    }
    if (stored.builder_version != requested.builder_version) {
        if (reason) *reason = "builder version differs";
        return false;
    }
    return true;
}

LectDatabaseIdentity make_identity_for_robot(const Robot& robot,
                                             const std::vector<Interval>& root_intervals,
                                             const SplitPolicyDescriptor& split_policy,
                                             bool canonical_mode,
                                             std::string endpoint_descriptor,
                                             std::string envelope_descriptor,
                                             std::string payload_layout,
                                             std::string builder_version) {
    LectDatabaseIdentity identity;
    identity.robot_fingerprint = robot.fingerprint();
    identity.root_domain_fingerprint = fingerprint_intervals(root_intervals);
    identity.split_policy_hash = split_policy_hash(split_policy);
    identity.canonical_mode = canonical_mode;
    identity.split_policy_descriptor = split_policy_descriptor(split_policy);
    if (!endpoint_descriptor.empty()) {
        identity.endpoint_descriptor = std::move(endpoint_descriptor);
    }
    if (!envelope_descriptor.empty()) {
        identity.envelope_descriptor = std::move(envelope_descriptor);
    }
    if (!payload_layout.empty()) {
        identity.payload_layout = std::move(payload_layout);
    }
    identity.builder_version = std::move(builder_version);
    return identity;
}

}  // namespace rbf::lect_database
