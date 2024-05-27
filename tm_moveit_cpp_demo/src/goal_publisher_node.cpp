#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <map>
#include <string>

class GoalPublisherNode : public rclcpp::Node
{
public:
  GoalPublisherNode()
    : Node("goal_publisher_node"),
      current_left_goal_index_(0),
      current_right_goal_index_(0),
      left_goal_reached_(true),
      right_goal_reached_(true)
  {
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_statesLR", 10, std::bind(&GoalPublisherNode::jointStateCallback, this, std::placeholders::_1));
    goal_left_publisher_ = this->create_publisher<std_msgs::msg::String>("/goal_left", 10);
    goal_right_publisher_ = this->create_publisher<std_msgs::msg::String>("/goal_right", 10);
    
    // Initialize goal positions for left and right arms
    left_arm_goals_ = {
        {"lefthome", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {"leftready1", {0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0}},
        {"leftready2", {0.0, 0.0, 1.5708, -1.5708, 1.5708, 0.0}},
        {"leftready3", {0.0, 0.0, 1.5708, 1.5708, -1.5708, 0.0}}
    };

    right_arm_goals_ = {
        {"righthome", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
        {"rightready1", {0.0, 0.0, 1.5708, 0.0, 1.5708, 0.0}},
        {"rightready2", {0.0, 0.0, 1.5708, -1.5708, 1.5708, 0.0}},
        {"rightready3", {0.0, 0.0, 1.5708, 1.5708, -1.5708, 0.0}}
    };

    left_arm_goal_sequence_ = {"lefthome", "leftready1", "lefthome", "leftready2"};
    right_arm_goal_sequence_ = {"righthome", "rightready1", "righthome", "rightready2"};

    position_tolerance_ = 0.001;
    velocity_tolerance_ = 0.001;

    // Publicação inicial dos goals
    publishNextLeftGoal();
    publishNextRightGoal();
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    bool left_positions_close = true;
    bool left_velocities_near_zero = true;
    bool right_positions_close = true;
    bool right_velocities_near_zero = true;

    if (current_left_goal_index_ < left_arm_goal_sequence_.size())
    {
      const std::string& current_left_goal = left_arm_goal_sequence_[current_left_goal_index_ - 1];
      for (size_t i = 0; i < 6; ++i)
      {
        if (std::abs(msg->position[i] - left_arm_goals_.at(current_left_goal)[i]) > position_tolerance_)
        {
          left_positions_close = false;
        }
        if (std::abs(msg->velocity[i]) > velocity_tolerance_)
        {
          left_velocities_near_zero = false;
        }
      }
    }

    if (current_right_goal_index_ < right_arm_goal_sequence_.size())
    {
      const std::string& current_right_goal = right_arm_goal_sequence_[current_right_goal_index_ - 1];
      for (size_t i = 6; i < 12; ++i)
      {
        if (std::abs(msg->position[i] - right_arm_goals_.at(current_right_goal)[i - 6]) > position_tolerance_)
        {
          right_positions_close = false;
        }
        if (std::abs(msg->velocity[i]) > velocity_tolerance_)
        {
          right_velocities_near_zero = false;
        }
      }
    }

    if (left_positions_close && left_velocities_near_zero)
    {
      left_goal_reached_ = true;
    }

    if (right_positions_close && right_velocities_near_zero)
    {
      right_goal_reached_ = true;
    }

    if (left_goal_reached_)
    {
      publishNextLeftGoal();
    }

    if (right_goal_reached_)
    {
      publishNextRightGoal();
    }
  }

  void publishNextLeftGoal()
  {
    if (current_left_goal_index_ < left_arm_goal_sequence_.size())
    {
      if (goal_left_publisher_->get_subscription_count() > 0)
      {
        const std::string& next_goal = left_arm_goal_sequence_[current_left_goal_index_];
        auto message = std_msgs::msg::String();
        message.data = next_goal;
        RCLCPP_INFO(this->get_logger(), "Publishing goal for left arm: %s", message.data.c_str());
        goal_left_publisher_->publish(message);
        left_goal_reached_ = false;
        current_left_goal_index_++;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "No subscribers for left arm goal");
      }
    }
  }

  void publishNextRightGoal()
  {
    if (current_right_goal_index_ < right_arm_goal_sequence_.size())
    {
      if (goal_right_publisher_->get_subscription_count() > 0)
      {
        const std::string& next_goal = right_arm_goal_sequence_[current_right_goal_index_];
        auto message = std_msgs::msg::String();
        message.data = next_goal;
        RCLCPP_INFO(this->get_logger(), "Publishing goal for right arm: %s", message.data.c_str());
        goal_right_publisher_->publish(message);
        right_goal_reached_ = false;
        current_right_goal_index_++;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "No subscribers for right arm goal");
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_left_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_right_publisher_;

  std::map<std::string, std::vector<double>> left_arm_goals_;
  std::map<std::string, std::vector<double>> right_arm_goals_;
  std::vector<std::string> left_arm_goal_sequence_;
  std::vector<std::string> right_arm_goal_sequence_;
  double position_tolerance_;
  double velocity_tolerance_;

  size_t current_left_goal_index_;
  size_t current_right_goal_index_;
  bool left_goal_reached_;
  bool right_goal_reached_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
