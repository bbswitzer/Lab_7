#include "../include/square.hpp"
#include <cmath>

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    // Create instance of joint target plan
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = 0;
    joint_targets["shoulder_lift_joint"] = 0;
    joint_targets["shoulder_pan_joint"] = 3.14159;
    joint_targets["wrist_1_joint"] = -3.14159/2;
    joint_targets["wrist_2_joint"] = -3.14159;
    joint_targets["wrist_3_joint"] = 0;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }
    


    // Create instance of joint target plan
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan1;

    std::map<std::string, double> joint_targets1;
    joint_targets1["elbow_joint"] = 0;
    joint_targets1["shoulder_lift_joint"] = 0;
    joint_targets1["shoulder_pan_joint"] = -3.14159;
    joint_targets1["wrist_1_joint"] = -3.14159/2;
    joint_targets1["wrist_2_joint"] = -3.14159;
    joint_targets1["wrist_3_joint"] = 0;

    bool joint_plan_success1;
    joint_plan_success1 = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan1, joint_targets1);

    if(joint_plan_success1){
        ROS_INFO("Moving to joint target");
        n.setParam("/record_pose", true);
        arm_move_group.execute(joint_plan1);
        n.setParam("/record_pose", false);
    }
    
}