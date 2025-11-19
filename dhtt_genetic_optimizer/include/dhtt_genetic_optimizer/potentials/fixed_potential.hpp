#pragma once

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/potential_type.hpp"

namespace dhtt_genetic_optimizer
{

/**
 * \brief Loads a fixed activation potential from the param server
 *
 * You cannot supply an argument to plugins at load time. Instead, we expect
 * the GA to set the parameter on the main tree server. If the parameter is not
 * set, return a default value 0.
 *
 * Two parameters to set: `FixedPotentialNodeNames` is an array of str node
 * names with or without the underscore and number suffix,
 * `FixedPotentialValues` is an array of real activation potentials between 0
 * and 1 inclusive.
 */
class FixedPotential : public dhtt::PotentialType
{
  public:
    FixedPotential();
    virtual ~FixedPotential() = default;

    double compute_activation_potential(dhtt::Node *container) override;
};
} // namespace dhtt_genetic_optimizer