#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ee4308_drone/smoother.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exe;

    // mapper node
    auto node = std::make_shared<ee4308::drone::ROSNodeSmoother>(
        "smoother" // node name
    );

    exe.add_node(node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}