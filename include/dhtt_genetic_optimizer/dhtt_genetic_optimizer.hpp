#pragma once

#include <pluginlib/class_list_macros.hpp>

#include "dhtt_genetic_optimizer/potentials/fixed_potential.hpp"

PLUGINLIB_EXPORT_CLASS(dhtt_genetic_optimizer::FixedPotential,
                       dhtt::PotentialType);