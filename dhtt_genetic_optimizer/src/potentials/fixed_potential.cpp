#include "dhtt_genetic_optimizer/potentials/fixed_potential.hpp"
#include "dhtt_genetic_optimizer/dhtt_genetic_optimizer.hpp"

namespace dhtt_genetic_optimizer
{
FixedPotential::FixedPotential() = default;

double FixedPotential::compute_activation_potential(dhtt::Node *container)
{
    constexpr auto DEFAULT = 0.0;

    std::vector<std::string> param_node_names;
    std::vector<double> param_node_values;

    // Check if parameters are actually set
    if (container->get_com_agg()->has_parameter(
            dhtt_genetic_optimizer::PARAM_NODE_NAMES))
    {
        param_node_names =
            container->get_com_agg()
                ->get_parameter_sync(dhtt_genetic_optimizer::PARAM_NODE_NAMES)
                .as_string_array();
    }
    else
    {
        std::stringstream ss;
        ss << "Returning 0.0 activation, parameter not set: "
           << dhtt_genetic_optimizer::PARAM_NODE_NAMES << std::endl;
        container->get_com_agg()->log_stream(dhtt::WARN, ss);
        return DEFAULT;
    }
    if (container->get_com_agg()->has_parameter(
            dhtt_genetic_optimizer::PARAM_NODE_VALUES))
    {
        param_node_values =
            container->get_com_agg()
                ->get_parameter_sync(dhtt_genetic_optimizer::PARAM_NODE_VALUES)
                .as_double_array();
    }
    else
    {
        std::stringstream ss;
        ss << "Returning 0.0 activation, parameter not set: "
           << dhtt_genetic_optimizer::PARAM_NODE_VALUES << std::endl;
        container->get_com_agg()->log_stream(dhtt::WARN, ss);
        return DEFAULT;
    }

    // Helper lambdas. In situ, the node name should have an underscore number
    // suffix
    auto find_index_with_suffix =
        [&param_node_names](const std::string &name_to_find)
    {
        auto it{std::find(param_node_names.cbegin(), param_node_names.cend(),
                          name_to_find)};
        return std::distance(param_node_names.cbegin(), it);
    };
    auto find_index_without_suffix =
        [&find_index_with_suffix](const std::string &name_to_find)
    {
        return find_index_with_suffix(
            name_to_find.substr(0, name_to_find.find('_')));
    };

    // Search
    auto index{find_index_with_suffix(container->get_node_name())};
    if (index ==
        std::distance(param_node_names.cbegin(), param_node_names.cend()))
    {
        // didn't find it with an underscore, lets broaden the search
        index = find_index_without_suffix(container->get_node_name());
    }

    if (index !=
        std::distance(param_node_names.cbegin(), param_node_names.cend()))
    {
        try
        {
            return param_node_values.at(index);
        }
        catch (std::out_of_range ex)
        {
            std::stringstream ss;
            ss << "Returning 0.0 activation, caught exception: " << ex.what()
               << std::endl;
            container->get_com_agg()->log_stream(dhtt::ERROR, ss);
            return DEFAULT;
        }
    }

    std::stringstream ss;
    ss << "Returning 0.0 activation, node name not found:"
       << container->get_node_name() << " in names: ";
    for (auto name : param_node_names)
    {
        ss << name << ' ';
    }
    ss << std::endl;

    container->get_com_agg()->log_stream(dhtt::ERROR, ss);
    return DEFAULT;
}
} // namespace dhtt_genetic_optimizer