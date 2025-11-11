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

    auto find_index_with_suffix =
        [&param_node_names](const std::string &name_to_find)
    {
        auto it{std::find(param_node_names.cbegin(), param_node_names.cend(),
                          name_to_find)};
        return std::distance(param_node_names.cbegin(), it);
    };

    auto find_index_without_suffix =
        [&param_node_names](const std::string &name_to_find)
    {
        auto pred = [name_to_find](std::string with_underscores)
        {
            auto substr =
                with_underscores.substr(0, with_underscores.find('_'));
            return name_to_find == substr;
        };

        auto it{std::find_if(param_node_names.cbegin(), param_node_names.cend(),
                             pred)};
        return std::distance(param_node_names.cbegin(), it);
    };

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
       << container->get_node_name() << std::endl;
    container->get_com_agg()->log_stream(dhtt::WARN, ss);
    return DEFAULT;
}
} // namespace dhtt_genetic_optimizer