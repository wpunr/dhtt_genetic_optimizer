#include "dhtt_genetic_optimizer/potentials/fixed_potential.hpp"

namespace dhtt_genetic_optimizer
{
FixedPotential::FixedPotential() = default;

double FixedPotential::compute_activation_potential(dhtt::Node *container)
{
    container; // unused
    return 1.0;
}
} // namespace dhtt_genetic_optimizer