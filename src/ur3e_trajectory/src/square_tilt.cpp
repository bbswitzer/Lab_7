#include "../include/square.hpp"
#include "cmath";

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.02;
    planning_options.goal_orientation_tolerance = 0.02;
    planning_options.goal_joint_tolerance = 0.02;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the square trajectory here.
    // Create instance of joint target plan
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = M_PI_2;
    joint_targets["shoulder_lift_joint"] = -M_PI_2;
    joint_targets["shoulder_pan_joint"] = -M_PI_2;
    joint_targets["wrist_1_joint"] = -M_PI_2;
    joint_targets["wrist_2_joint"] = -M_PI_2;
    joint_targets["wrist_3_joint"] = 0;

    bool joint_plan_success;
    joint_plan_success = ArmController::planToJointTargets(planning_options, arm_move_group, joint_plan, joint_targets);

    if(joint_plan_success){
        ROS_INFO("Moving to joint target");
        arm_move_group.execute(joint_plan);
    }

    // Create instance of pose target plan
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    auto current_pose {arm_move_group.getCurrentPose().pose};

    double size = 0.5;
    double theta = 0.1;
    geometry_msgs::Pose pose_target {current_pose};
    pose_target.position.y = size*cos(theta);
    pose_target.position.x = 0;
    pose_target.position.z = 0.9;


    bool pose_plan_success;
    std::string reference_frame = "world";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }

    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose end_pose = start_pose;
    end_pose.position.y = 0;
    end_pose.position.x = size;
    end_pose.position.z = 0.9+size*sin(theta);

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(end_pose);
    end_pose.position.y = -size*cos(theta);
    end_pose.position.x = 0;
    end_pose.position.z = 0.9+size*sin(theta)+size*sin(theta);

    // Define waypoints for the cartesian path
    waypoints.push_back(end_pose);

    end_pose.position.y = 0;
    end_pose.position.x = -size;
    end_pose.position.z = 0.9+size*sin(theta);

    // Define waypoints for the cartesian path
    waypoints.push_back(end_pose);


    end_pose.position.y = size*cos(theta);
    end_pose.position.x = 0;
    end_pose.position.z = 0.9;

    // Define waypoints for the cartesian path
    waypoints.push_back(end_pose);



    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);
    
}