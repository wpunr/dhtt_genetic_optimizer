#pragma once

#include <pluginlib/class_list_macros.hpp>

#include "dhtt_genetic_optimizer/potentials/fixed_potential.hpp"

namespace dhtt_genetic_optimizer
{
constexpr auto PARAM_NODE_NAMES{"FixedPotentialNodeNames"};
constexpr auto PARAM_NODE_VALUES{"FixedPotentialValues"};
} // namespace dhtt_genetic_optimizer

PLUGINLIB_EXPORT_CLASS(dhtt_genetic_optimizer::FixedPotential,
                       dhtt::PotentialType);