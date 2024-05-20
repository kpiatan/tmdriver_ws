/*********************************************************************
 *  run_moveit_cpp.cpp
 * 
 *  Various portions of the code are based on original source from 
 *  PickNik Inc.
 *  and are used in accordance with the following license. */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
   Desc: A simple demo node running MoveItCpp for planning and execution
*/

#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_cpp_demo");

class MoveItCppDemo
{
public:
  MoveItCppDemo(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    , robot_state_publisher_(node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1))
  {
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize MoveItCpp");
    //moveit_cpp_ = std::make_shared<moveit::planning_interface::MoveItCpp>(node_);
    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node_);
    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();  // let RViz display query PlanningScene
    moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(100);

    RCLCPP_INFO(LOGGER, "Initialize PlanningComponent");
    //moveit::planning_interface::PlanningComponent arm_left("left_tmr_arm", moveit_cpp_);
    //moveit::planning_interface::PlanningComponent arm_right("right_tmr_arm", moveit_cpp_);
    moveit_cpp::PlanningComponent arm_left("left_tmr_arm", moveit_cpp_);
    moveit_cpp::PlanningComponent arm_right("right_tmr_arm", moveit_cpp_);

    // A little delay before running the plan
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goals");
    //arm_left.setGoal("lefthome");
    //arm_left.setGoal("leftready1");
    //arm_left.setGoal("leftready2");
    //arm_left.setGoal("leftready3");

    //arm_right.setGoal("righthome");
    //arm_right.setGoal("rightready1");
    //arm_right.setGoal("rightready2");
    //arm_right.setGoal("rightready3");
    
    //arm_left.setGoal("lefthome");   
    arm_right.setGoal("righthome");  

    // Define a sequência de goals para o braço esquerdo
    std::vector<std::string> left_arm_goals = {"lefthome", "leftready1"};

    // Itera sobre os goals do braço esquerdo
    for (const auto& goal : left_arm_goals)
    {
      // Define o goal para o braço esquerdo
      RCLCPP_INFO(LOGGER, "Definindo o goal para o braço esquerdo: %s", goal.c_str());
      arm_left.setGoal(goal);

      // Executa o plano para o braço esquerdo
      RCLCPP_INFO(LOGGER, "(ESQUERDA) Planejando para o goal: %s", goal.c_str());
      auto plan_solution_left = arm_left.plan();
      if (plan_solution_left)
      {
        RCLCPP_INFO(LOGGER, "Executando o plano para o braço esquerdo");
        arm_left.execute();
      }
      else
      {
        RCLCPP_ERROR(LOGGER, "Falha ao planejar para o goal: %s", goal.c_str());
        // Lida com a falha no planejamento (por exemplo, tenta o próximo goal ou interrompe a execução)
      }
    }

    // Run actual plan
    // RCLCPP_INFO(LOGGER, "(LEFT) Plan to goal");
    // auto plan_solution_left = arm_left.plan();
    // if (plan_solution_left)
    // {
    //   RCLCPP_INFO(LOGGER, "arm_left.execute()");
    //   arm_left.execute();
    // }

    RCLCPP_INFO(LOGGER, "(RIGHT) Plan to goal");
    auto plan_solution_right = arm_right.plan();
    if (plan_solution_right)
    {
      RCLCPP_INFO(LOGGER, "arm_right.execute()");
      arm_right.execute();
    }

    //Below, we simply use a long delay to wait for the previous motion to complete.
    /*rclcpp::sleep_for(std::chrono::seconds(10));   

    // Set joint state goal
    RCLCPP_INFO(LOGGER, "Set goal (home)");
    arm.setGoal("home");-

    // Run actual plan
    RCLCPP_INFO(LOGGER, "Plan to goal");
    plan_solution = arm.plan();
    if (plan_solution)
    {
      RCLCPP_INFO(LOGGER, "arm.execute()");
      arm.execute();
    }*/
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  //moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
};

int main(int argc, char** argv)
{
  RCLCPP_INFO(LOGGER, "Initialize node");
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);

  MoveItCppDemo demo(node);
  std::thread run_demo([&demo]() {
    // Let RViz initialize before running demo
    // TODO(henningkayser): use lifecycle events to launch node
    rclcpp::sleep_for(std::chrono::seconds(5));
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();

  return 0;
}
