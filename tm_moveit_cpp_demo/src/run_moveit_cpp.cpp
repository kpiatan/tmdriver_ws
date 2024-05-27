#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo : public rclcpp::Node
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : Node("moveit_cpp_demo_node"), node_(node)
  {
    robot_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);

    goal_left_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "/goal_left", 10, std::bind(&MoveItCppDemo::goalLeftCallback, this, std::placeholders::_1));
    goal_right_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "/goal_right", 10, std::bind(&MoveItCppDemo::goalRightCallback, this, std::placeholders::_1));
  
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    arm_left_ = std::make_shared<moveit_cpp::PlanningComponent>("left_tmr_arm", moveit_cpp_);
    arm_right_ = std::make_shared<moveit_cpp::PlanningComponent>("right_tmr_arm", moveit_cpp_);
  }

private:

  void goalLeftCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(LOGGER, "Setting goal for left arm: %s", msg->data.c_str());
    arm_left_->setGoal(msg->data);

    // Plan and execute for the left arm
    RCLCPP_INFO(LOGGER, "Planning for left arm goal: %s", msg->data.c_str());
    auto plan_solution_left = arm_left_->plan();
    if (plan_solution_left)
    {
      RCLCPP_INFO(LOGGER, "Executing plan for left arm");
      arm_left_->execute();
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to plan for left arm goal: %s", msg->data.c_str());
    }
  }

  void goalRightCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(LOGGER, "Setting goal for right arm: %s", msg->data.c_str());
    arm_right_->setGoal(msg->data);

    // Plan and execute for the right arm
    RCLCPP_INFO(LOGGER, "Planning for right arm goal: %s", msg->data.c_str());
    auto plan_solution_right = arm_right_->plan();
    if (plan_solution_right)
    {
      RCLCPP_INFO(LOGGER, "Executing plan for right arm");
      arm_right_->execute();
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to plan for right arm goal: %s", msg->data.c_str());
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_left_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_right_subscriber_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  std::shared_ptr<moveit_cpp::PlanningComponent> arm_left_;
  std::shared_ptr<moveit_cpp::PlanningComponent> arm_right_;

};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
