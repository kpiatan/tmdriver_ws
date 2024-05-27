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
    joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_statesLR", 10, std::bind(&MoveItCppDemo::jointStateCallback, this, std::placeholders::_1));
  
    // Define goal positions and tolerances
    goal_positions_ = {1.0, 0.5, -1.0, 1.5, 0.0, -0.5}; // Example goal positions for the joints
    position_tolerance_ = 0.1;
    velocity_tolerance_ = 0.01;
  
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    moveit_cpp::PlanningComponent arm_left("left_tmr_arm", moveit_cpp_);
    moveit_cpp::PlanningComponent arm_right("right_tmr_arm", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goals");

    // Define a sequence of goals for the left arm
    std::vector<std::string> left_arm_goals = {"lefthome", "leftready1"};

    // Iterate over the goals for the left arm
    for (const auto& goal : left_arm_goals)
    {
      // Set the goal for the left arm
      RCLCPP_INFO(LOGGER, "Setting goal for left arm: %s", goal.c_str());
      arm_left.setGoal(goal);

      // Plan and execute for the left arm
      RCLCPP_INFO(LOGGER, "Planning for left arm goal: %s", goal.c_str());
      auto plan_solution_left = arm_left.plan();
      if (plan_solution_left)
      {
        RCLCPP_INFO(LOGGER, "Executing plan for left arm");
        arm_left.execute();
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Failed to plan for left arm goal: %s", goal.c_str());
      }
    }

    // Define a sequence of goals for the right arm
    std::vector<std::string> right_arm_goals = {"rightready1", "rightready2"};

    // Iterate over the goals for the right arm
    for (const auto& goal : right_arm_goals)
    {
      // Set the goal for the right arm
      RCLCPP_INFO(LOGGER, "Setting goal for right arm: %s", goal.c_str());
      arm_right.setGoal(goal);

      // Plan and execute for the right arm
      RCLCPP_INFO(LOGGER, "Planning for right arm goal: %s", goal.c_str());
      auto plan_solution_right = arm_right.plan();
      if (plan_solution_right)
      {
        RCLCPP_INFO(LOGGER, "Executing plan for right arm");
        arm_right.execute();
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Failed to plan for right arm goal: %s", goal.c_str());
      }
    }
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool positions_close = true;
    bool velocities_near_zero = true;

    for (size_t i = 0; i < msg->position.size(); ++i)
    {
      if (std::abs(msg->position[i] - goal_positions_[i]) > position_tolerance_)
      {
        positions_close = false;
      }
      if (std::abs(msg->velocity[i]) > velocity_tolerance_)
      {
        velocities_near_zero = false;
      }
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;

  std::vector<double> goal_positions_;
  double position_tolerance_;
  double velocity_tolerance_;
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
