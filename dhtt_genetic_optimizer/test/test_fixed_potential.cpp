#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_loader.hpp>

#include "dhtt/server/communication_aggregator.hpp"
#include "dhtt/server/main_server.hpp"
#include "dhtt/tree/node.hpp"
#include "dhtt/tree/potential_type.hpp"
#include "dhtt_genetic_optimizer/dhtt_genetic_optimizer.hpp"

#include "dhtt_genetic_optimizer/potentials/fixed_potential.hpp"

/*** Begin Copied from dhtt_plugins/test/potential_test.cpp ***/

class TestMainServer : public dhtt::MainServer
{
  public:
    TestMainServer(
        std::string node_name,
        std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner,
        bool slow = false)
        : dhtt::MainServer(node_name, spinner, slow) {};

    std::shared_ptr<dhtt_msgs::srv::ModifyRequest::Response>
    add_from_file(std::string path)
    {
        const auto req =
            std::make_shared<dhtt_msgs::srv::ModifyRequest::Request>();
        auto res = std::make_shared<dhtt_msgs::srv::ModifyRequest::Response>();

        req->type = dhtt_msgs::srv::ModifyRequest::Request::ADD_FROM_FILE;
        req->to_modify = {"ROOT_0"};
        req->force = true;
        req->to_add = path;

        this->modify_callback(req, res);

        return res;
    }

    std::unordered_map<std::string, std::shared_ptr<dhtt::Node>>
    test_get_node_map() const
    {
        return this->get_node_map();
    };

    std::shared_ptr<dhtt::CommunicationAggregator> test_get_com_agg() const
    {
        return this->get_com_agg();
    }
};

class TestNode : public dhtt::Node
{
  public:
    TestNode(std::shared_ptr<dhtt::CommunicationAggregator> com_agg,
             std::string name, std::string type,
             std::vector<std::string> params, std::string parent_name,
             std::string socket_type = "dhtt_plugins::PtrBranchSocket",
             std::string goitr_type = "",
             std::string potential_type = "dhtt_plugins::EfficiencyPotential")
        : dhtt::Node(com_agg, name, type, params, parent_name, socket_type,
                     goitr_type, potential_type)
    {
    }
    ~TestNode() override = default;

    size_t fake_num_resources = 0;
    size_t fake_locked_resources = 0;
    size_t fake_subtree_resources = 0;

    void check_fake_values() const
    {
        ASSERT_LE(fake_locked_resources, fake_num_resources);
        ASSERT_LE(fake_subtree_resources, fake_num_resources);
    }

    std::vector<dhtt_msgs::msg::Resource> get_resource_state() override
    {
        check_fake_values();
        std::vector<dhtt_msgs::msg::Resource> res(fake_num_resources,
                                                  dhtt_msgs::msg::Resource());
        for (auto i = 0; i < fake_locked_resources; ++i)
        {
            res[i].locked = true;
        }
        return res;
    }

    int get_subtree_resources() override
    {
        check_fake_values();
        return static_cast<int>(fake_subtree_resources);
    }
};

class TestMainServerF : public testing::Test
{
  public:
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner;
    std::shared_ptr<TestMainServer> test_main_server;

  protected:
    virtual void SetUp() override
    {
        this->spinner =
            std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        this->test_main_server = std::make_shared<TestMainServer>(
            "dHTT_server", this->spinner, false);

        RCLCPP_INFO(this->test_main_server->get_logger(),
                    "Fixture Server started...");
        this->spinner->add_node(this->test_main_server);
    }

    virtual void TearDown() override {}
};

TEST_F(TestMainServerF, test_fixture)
{
    RCLCPP_INFO(test_main_server->get_logger(), "Foo");
    RCLCPP_INFO(test_main_server->test_get_com_agg()->get_logger(), "Bar");
    std::cout << test_main_server->test_get_node_map().begin()->first
              << std::endl;
}

TEST_F(TestMainServerF, test_make_node)
{
    auto node = std::make_shared<dhtt::Node>(
        test_main_server->test_get_com_agg(), "Foo",
        "dhtt_plugins::TestBehavior",
        std::vector<std::string>({"activation_potential: 0.5"}), "ROOT_0",
        "dhtt_plugins::PtrBranchSocket", "",
        "dhtt_plugins::EfficiencyPotential");

    auto res = node->get_logic()->get_perceived_efficiency(node.get());
    std::cout << res << std::endl;
    ASSERT_DOUBLE_EQ(res, 0.5);
}

/*** End Copied from dhtt_plugins/test/potential_test.cpp ***/

class TestFixedPotentialF : public TestMainServerF
{
  public:
    std::shared_ptr<dhtt::Node> node;
    pluginlib::UniquePtr<dhtt::PotentialType> potential_plugin;

  protected:
    void SetUp() override
    {
        TestMainServerF::SetUp();

        this->node = std::make_shared<dhtt::Node>(
            test_main_server->test_get_com_agg(), "Bar_1",
            "dhtt_plugins::TestBehavior", std::vector<std::string>{}, "ROOT_0",
            "dhtt_plugins::PtrBranchSocket", "",
            "dhtt_plugins::EfficiencyPotential");
        // Note that we are not having dhtt::Node load the code under test, so
        // ignore EfficiencyPotential

        this->potential_plugin =
            pluginlib::ClassLoader<dhtt::PotentialType>("dhtt",
                                                        "dhtt::PotentialType")
                .createUniqueInstance("dhtt_genetic_optimizer::FixedPotential");

        ASSERT_TRUE(potential_plugin);
    }
};

TEST_F(TestFixedPotentialF, test_FixedPotential)
{
    // No parameter set, check default return
    double res{potential_plugin->compute_activation_potential(node.get())};
    std::cout << res << std::endl;
    ASSERT_DOUBLE_EQ(res, 0.0);
}

TEST_F(TestFixedPotentialF, test_FixedPotential_bad1)
{
    // Set parameters that don't apply to this node
    node->get_com_agg()->set_parameter_sync(
        rclcpp::Parameter(dhtt_genetic_optimizer::PARAM_NODE_NAMES,
                          std::vector<std::string>{"Foo_2", "Foo_3"}));
    node->get_com_agg()->set_parameter_sync(
        rclcpp::Parameter(dhtt_genetic_optimizer::PARAM_NODE_VALUES,
                          std::vector<double>{0.1, 0.5}));
    auto res{potential_plugin->compute_activation_potential(node.get())};
    std::cout << res << std::endl;
    ASSERT_DOUBLE_EQ(res, 0.0);
}

TEST_F(TestFixedPotentialF, test_FixedPotential_bad2)
{
    // Set parameters but forget to do the other, should catch and return
    // default
    node->get_com_agg()->set_parameter_sync(rclcpp::Parameter(
        dhtt_genetic_optimizer::PARAM_NODE_NAMES,
        std::vector<std::string>{"Foo_2", "Foo_3", node->get_node_name()}));
    node->get_com_agg()->set_parameter_sync(rclcpp::Parameter(
        dhtt_genetic_optimizer::PARAM_NODE_VALUES,
        std::vector<double>{0.1, 0.5})); // missing its value here
    auto res{potential_plugin->compute_activation_potential(node.get())};
    std::cout << res << std::endl;
    ASSERT_DOUBLE_EQ(res, 0.0);
}

TEST_F(TestFixedPotentialF, test_FixedPotential_happy)
{
    // Set parameters correctly
    // node->get_node_name() should have the underscore suffix
    ASSERT_EQ(node->get_node_name(), "Bar_1");
    node->get_com_agg()->set_parameter_sync(rclcpp::Parameter(
        dhtt_genetic_optimizer::PARAM_NODE_NAMES,
        std::vector<std::string>{"Foo_2", "Foo_3", node->get_node_name()}));
    node->get_com_agg()->set_parameter_sync(
        rclcpp::Parameter(dhtt_genetic_optimizer::PARAM_NODE_VALUES,
                          std::vector<double>{0.1, 0.5, 0.9}));
    auto res{potential_plugin->compute_activation_potential(node.get())};
    std::cout << res << std::endl;
    ASSERT_DOUBLE_EQ(res, 0.9);
}

TEST_F(TestFixedPotentialF, test_FixedPotential_happy_underscore)
{
    // Try it without the underscore suffix
    ASSERT_EQ(node->get_node_name(), "Bar_1");
    auto without_suffix = node->get_node_name().substr(0, node->get_node_name().find('_'));
    ASSERT_EQ(without_suffix, "Bar");
    node->get_com_agg()->set_parameter_sync(rclcpp::Parameter(
        dhtt_genetic_optimizer::PARAM_NODE_NAMES,
        std::vector<std::string>{"Foo_2", "Foo_3", without_suffix}));
    node->get_com_agg()->set_parameter_sync(
        rclcpp::Parameter(dhtt_genetic_optimizer::PARAM_NODE_VALUES,
                          std::vector<double>{0.1, 0.5, 0.9}));
    auto res{potential_plugin->compute_activation_potential(node.get())};
    std::cout << res << std::endl;
    ASSERT_DOUBLE_EQ(res, 0.9);
}

int main(int argc, char **argv)
{
    rclcpp::init(0, nullptr);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return res;
}