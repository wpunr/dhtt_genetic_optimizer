#pragma once

#include "dhtt/tree/node.hpp"
#include "dhtt/tree/potential_type.hpp"

namespace dhtt_genetic_optimizer
{
class FixedPotential : public dhtt::PotentialType
{
  public:
    FixedPotential();
    virtual ~FixedPotential() = default;

    double compute_activation_potential(dhtt::Node *container) override;
};
} // namespace dhtt_genetic_optimizer