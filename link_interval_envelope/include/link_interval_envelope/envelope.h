#pragma once
/// @file envelope.h
/// @brief Public link-envelope configuration, result, and collision API.

#include <sbf/envelope/envelope_collision.h>
#include <sbf/envelope/envelope_type.h>

namespace link_interval_envelope {

using ::rbf::CollisionResultKind;
using ::rbf::EnvelopeCollisionMode;
using ::rbf::EnvelopeCollisionOptions;
using ::rbf::EnvelopeCollisionStats;
using ::rbf::EnvelopeType;
using ::rbf::EnvelopeTypeConfig;
using ::rbf::KdopConfig;
using ::rbf::KdopDirectionSet;
using ::rbf::LinkEnvelope;
using ::rbf::SupportHullConfig;

using ::rbf::collide_envelope_aabbs;
using ::rbf::compute_link_envelope;
using ::rbf::envelope_type_name;

}  // namespace link_interval_envelope
