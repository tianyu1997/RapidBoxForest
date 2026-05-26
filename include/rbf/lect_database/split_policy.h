#pragma once

#include <rbf/lect_database/types.h>

#include <string>

namespace rbf::lect_database {

enum class SplitStrategy : std::uint8_t {
    RoundRobin = 0,
    WidestRoot = 1,
    AAFKVolumeMin = 2,
};

struct SplitPolicyDescriptor {
    SplitStrategy strategy = SplitStrategy::RoundRobin;
    double min_width = 0.0;
    bool midpoint = true;
    bool deterministic_tie_break = true;
    std::string dimension_schedule_hash;
    std::vector<int> depth_dimensions;
};

const char* split_strategy_name(SplitStrategy strategy) noexcept;
std::string split_policy_descriptor(const SplitPolicyDescriptor& descriptor);
std::uint64_t split_policy_hash(const SplitPolicyDescriptor& descriptor);

class SplitPolicy {
public:
    explicit SplitPolicy(SplitPolicyDescriptor descriptor = {});

    const SplitPolicyDescriptor& descriptor() const noexcept { return descriptor_; }
    std::uint64_t hash() const { return split_policy_hash(descriptor_); }
    int choose_dimension(const std::vector<Interval>& root_intervals,
                         const std::vector<Interval>& node_intervals,
                         int depth) const;
    double choose_split_value(const Interval& interval) const;

private:
    SplitPolicyDescriptor descriptor_;
};

}  // namespace rbf::lect_database
