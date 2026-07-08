#include "grower_internal.h"

#include <SBF/oracle.h>
#include <SBF/runtime.h>

#include <algorithm>

namespace rbf {

bool allow_box_commit(BoxOracle& oracle,
                      FindFreeBoxResult& result,
                      BoxCommitPolicy policy,
                      StageContext& context) {
    if (result.validation_detail.safety_status == BoxSafetyStatus::CertifiedFree) {
        return true;
    }
    if (result.validation_detail.safety_status != BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("grower.commit_rejected_unknown_status");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitCertifiedOnly) {
        context.diagnostics().add_counter("grower.commit_rejected_provisional");
        return false;
    }
    if (policy == BoxCommitPolicy::CommitProvisionalAllowed) {
        context.diagnostics().add_counter("grower.commit_provisional_allowed");
        return true;
    }
    if (policy == BoxCommitPolicy::AuditBeforeCommit) {
        context.diagnostics().add_counter("grower.commit_audit_attempted");
        if (oracle.validate_intervals(result.intervals)) {
            result.validation_detail.safety_status = BoxSafetyStatus::CertifiedFree;
            result.validation_detail.strict_audit_required = false;
            context.diagnostics().add_counter("grower.commit_audit_success");
            return true;
        }
        context.diagnostics().add_counter("grower.commit_audit_failed");
        return false;
    }
    return false;
}

void set_grower_max_diagnostic(StageContext& context, const std::string& key, double value) {
    context.diagnostics().set_value(key, std::max(context.diagnostics().value(key), value));
}

void record_committed_box_stats(StageContext& context, const BoxNode& box) {
    context.diagnostics().add_counter("grower.committed_box_volume_sum", box.volume);
    set_grower_max_diagnostic(context, "grower.committed_box_volume_max", box.volume);
    if (box.safety_status == BoxSafetyStatus::CertifiedFree) {
        context.diagnostics().add_counter("grower.certified_boxes_committed");
        context.diagnostics().add_counter("grower.certified_box_volume_sum", box.volume);
    } else if (box.safety_status == BoxSafetyStatus::ProvisionalFree) {
        context.diagnostics().add_counter("grower.provisional_boxes_committed");
        context.diagnostics().add_counter("grower.provisional_box_volume_sum", box.volume);
    }
    if (box.strict_audit_required) {
        context.diagnostics().add_counter("grower.strict_audit_required_boxes");
        context.diagnostics().add_counter("grower.strict_audit_required_volume_sum", box.volume);
    }
}

OracleNodeId find_leaf_containing(BoxOracle& oracle, const Eigen::Ref<const Eigen::VectorXd>& q) {
    if (q.size() != oracle.n_dims() || !oracle.contains_point(oracle.root_node(), q)) {
        return kInvalidOracleNodeId;
    }
    OracleNodeId node = oracle.root_node();
    while (!oracle.is_leaf(node)) {
        node = oracle.child_containing_point(node, q);
        if (node == kInvalidOracleNodeId) {
            break;
        }
    }
    return node;
}

}  // namespace rbf
