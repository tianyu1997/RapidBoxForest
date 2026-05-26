#include <rbf/lect_database.h>

#ifdef RBF_PLANNING_DETAIL_INCLUDED
#error "rbf/lect_database.h must not include rbf/planning/detail.h"
#endif

#include <type_traits>

static_assert(std::is_class_v<rbf::lect_database::LectDatabase>);
static_assert(std::is_same_v<rbf::lect_database::NodeId, std::uint64_t>);
static_assert(std::is_class_v<rbf::lect_database::LectDatabaseIdentity>);
static_assert(std::is_class_v<rbf::lect_database::SplitPolicyDescriptor>);
