#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <ee4308_turtle/behavior.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    // mapper node
    auto node = std::make_shared<ee4308::turtle::ROSNodeBehavior>(
        "behavior" // node name
    );

    exe.add_node(node);
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
