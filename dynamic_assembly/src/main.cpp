#include <rclcpp/rclcpp.hpp>
#include "dynamic_assembly/ekf_filter_node.hpp"
#include "dynamic_assembly/apf_planner_node.hpp"
#include "dynamic_assembly/mpc_planner_node.hpp"

int main(int argc, char** argv) {
    // 1. 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 2. 设置节点选项 
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    // 3. 实例化节点
    auto ekf_node = std::make_shared<dynamic_assembly::EkfFilterNode>(options);
    auto apf_node = std::make_shared<dynamic_assembly::ApfPlannerNode>(options);
    auto mpc_node = std::make_shared<dynamic_assembly::MpcPlannerNode>(options);

    // 4. 初始化 MoveIt 组件
    apf_node->initialize_APF();
    mpc_node->initialize_MPC();

    // 5. 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;

    // 6. 把节点加入执行器
    executor.add_node(ekf_node);
    executor.add_node(apf_node);
    executor.add_node(mpc_node);

    RCLCPP_INFO(rclcpp::get_logger("SystemRunner"), "Dynamic Assembly System Started.");

    // spin自动分配后台线程
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
