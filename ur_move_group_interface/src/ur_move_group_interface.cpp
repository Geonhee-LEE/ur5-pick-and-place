/*
 
 *  UR5 move group interface
  
 *
 *  Created on: 2019. 05. 01.
 *      Author: Geonhee-LEE
 */

 

#include "ur_move_group_interface.h"

using namespace std;
using namespace moveit::planning_interface;
using namespace moveit_visual_tools;

namespace rvt = rviz_visual_tools;

URMoveGroup::URMoveGroup()
{
    gripper_pub = nh_.advertise<robotiq_2f_gripper_msgs::RobotiqGripperCommand>("gripper_command", 1000);


    PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    static MoveGroupInterface move_group(PLANNING_GROUP);
    static ::MoveItVisualTools visual_tools("base_link");

    init_move_group(move_group);
    init_visualization(visual_tools);
    start(move_group, visual_tools);
}


URMoveGroup::~URMoveGroup()
{
}
 

void URMoveGroup::init_move_group(MoveGroupInterface& move_group)
{
    planning_frame = move_group.getPlanningFrame();
    endeffector_frame = move_group.getEndEffectorLink();   
    
    // Getting Basic Information    
    ROS_INFO_STREAM("PLANNING_GROUP: " << PLANNING_GROUP);
    // We can print the name of the reference frame for this robot.
    ROS_INFO_STREAM("Planning frame: " << planning_frame);
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_STREAM("End effector frame: " << endeffector_frame);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Create init workspace
    create_worktop(move_group.getPlanningFrame());
}
        
        
void URMoveGroup::init_visualization(MoveItVisualTools& visual_tools)
{
    // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
    namespace rvt = rviz_visual_tools;
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    _text_pose = Eigen::Affine3d::Identity();
    _text_pose.translation().z() = 1.75;
    visual_tools.publishText( _text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    // Start the demo
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");
}


void URMoveGroup::create_worktop(std::string collision_id)
{
    ROS_INFO("create_worktop");

    worktop_obj.header.frame_id = collision_id;

    // The id of the object is used to identify it.
    worktop_obj.id = "worktop";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 1;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = -0.0;
    box_pose.position.z = -0.55;

    worktop_obj.primitives.push_back(primitive);
    worktop_obj.primitive_poses.push_back(box_pose);
    worktop_obj.operation = worktop_obj.ADD;

    collision_objects.push_back(worktop_obj);
        
    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);
}


void URMoveGroup::plan_to_goal(MoveGroupInterface& move_group, geometry_msgs::Pose target_pose)
{
    // Planning to a Pose goal
    // We can plan a motion for this group to a desired pose for the end-effector.
    move_group.setPoseTarget(target_pose);
    
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group to actually move the robot.
    bool success = (move_group.plan(_plan) == MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    ////// Moving to a pose goal //////
    // Moving to a pose goal is similar to the step above except we now use the move() function. Note that
    // the pose goal we had set earlier is still active and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    /* move_group.move(); */
}

void URMoveGroup::plan_to_goal(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, geometry_msgs::Pose target_pose)
{
    // Planning to a Pose goal
    // We can plan a motion for this group to a desired pose for the end-effector.
    move_group.setPoseTarget(target_pose);
    
    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group to actually move the robot.
    bool success = (move_group.plan(_plan) == MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose, "pose1");
    visual_tools.publishText( _text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    ////// Moving to a pose goal //////
    // Moving to a pose goal is similar to the step above except we now use the move() function. Note that
    // the pose goal we had set earlier is still active and so the robot will try to move to that goal. We will
    // not use that function in this tutorial since it is a blocking function and requires a controller to be active
    // and report success on execution of a trajectory.

    /* Uncomment below line when working with a real robot */
    /* move_group.move(); */
}

void URMoveGroup::plan_joint_space(MoveGroupInterface& move_group, vector<double> target_joint_group_positions)
{
    //// Planning to a joint-space goal
    // Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    
    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    move_group.setJointValueTarget(target_joint_group_positions);

    bool success = (move_group.plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

}
        
void URMoveGroup::plan_joint_space(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, vector<double> target_joint_group_positions)
{
    //// Planning to a joint-space goal
    // Let's set a joint space goal and move towards it.  This will replace the pose target we set above.
    // To start, we'll create an pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    
    // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
    move_group.setJointValueTarget(target_joint_group_positions);

    bool success = (move_group.plan(_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText( _text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");    
}

void URMoveGroup::update_current_state(MoveGroupInterface& move_group)
{
    // Update current state global variable
    current_state = move_group.getCurrentState();
    current_pose = move_group.getCurrentPose().pose;
}

std::vector<double> URMoveGroup::get_current_jointgroup(MoveGroupInterface& move_group)
{
    // Copy joint_model_group to the join group position 
    // target_joint_group_positions[0(base_link), ..., 5(wrist3)]
    std::vector<double> target_joint_group_positions;

    //Update current state
    update_current_state(move_group);
    current_state->copyJointGroupPositions(joint_model_group, target_joint_group_positions);

    return target_joint_group_positions;
}

void URMoveGroup::plan_with_path_constraint(MoveGroupInterface& move_group,  moveit_msgs::OrientationConstraint ocm, geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose)
{

  ////// Planning with Path Constraints
  // Path constraints can easily be specified for a link on the robot.

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already satisfies the path constraints. So, we need to set the start state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());

  start_state.setFromIK(joint_model_group, start_pose);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new start state that we have just created.
  move_group.setPoseTarget(target_pose);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan( _plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();

}
        
void URMoveGroup::plan_with_path_constraint(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, moveit_msgs::OrientationConstraint ocm, geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose)
{    

  ////// Planning with Path Constraints
  // Path constraints can easily be specified for a link on the robot.

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already satisfies the path constraints. So, we need to set the start state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  start_state.setFromIK(joint_model_group, start_pose);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new start state that we have just created.
  move_group.setPoseTarget(target_pose);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (move_group.plan( _plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose, "start");
  visual_tools.publishAxisLabeled(target_pose, "goal");
  visual_tools.publishText( _text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine( _plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // When done with the path constraint be sure to clear it.
  move_group.clearPathConstraints();

  // Since we set the start state we have to clear it before planning other paths
  move_group.setStartStateToCurrentState();
}


void URMoveGroup::plan_cartesian_space(MoveGroupInterface& move_group, std::vector<geometry_msgs::Pose>  waypoints)
{


}
        
void URMoveGroup::plan_cartesian_space(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, std::vector<geometry_msgs::Pose>  waypoints)
{    

    ////// Cartesian Paths
    // You can plan a Cartesian path directly by specifying a list of waypoints for the end-effector to go through. 
    // Note that we are starting from the new start state above.  
    // The initial pose (start state) does not need to be added to the waypoint list but adding it can help with visualizations

    // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
    // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
    // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
    move_group.setMaxVelocityScalingFactor(0.1);

    // We want the Cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in Cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
    // Warning - disabling the jump threshold while operating real hardware can cause
    // large unpredictable motions of redundant joints and could be a safety issue
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // Get robot trajectory to which joint speeds will be added
	robot_trajectory::RobotTrajectory robot_move_trajectory(move_group.getCurrentState()->getRobotModel(), "manipulator");
		
    // Second get a RobotTrajectory from trajectory
	robot_move_trajectory.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    // Visualize the plan in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(_text_pose, "Cartesian path", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
}

void URMoveGroup::send_gripper(robotiq_2f_gripper_msgs::RobotiqGripperCommand command)
{
    gripper_pub.publish(command);
}
    

void URMoveGroup::start(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools)
{
    gripper_cmd.position = 0;
    send_gripper(gripper_cmd);

    //Planning to a Pose goal
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.20;
    target_pose.position.y = -0.4;
    target_pose.position.z = 0.7;
    plan_to_goal(move_group, visual_tools, target_pose);

    //Planning to a joint-space goal
    std::vector<double> target_joint_group_positions = get_current_jointgroup(move_group);
    target_joint_group_positions[0] = -1.0;  // radians
    plan_joint_space(move_group, visual_tools, target_joint_group_positions);

    //Path constraint
    // Let's specify a path constraint and a pose goal for our group.
    // First define the path constraint.
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "tool0";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    geometry_msgs::Pose start_pose;
    start_pose.orientation.w = 1.0;
    start_pose.position.x = 0.3;
    start_pose.position.y = -0.35;
    start_pose.position.z = 0.5;
    plan_with_path_constraint(move_group, visual_tools, ocm, start_pose, target_pose);


    // Cartesian Paths
    update_current_state(move_group);
    geometry_msgs::Pose target_crts_pose = current_pose;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_crts_pose);
    target_crts_pose.position.z -= 0.2;
    waypoints.push_back(target_crts_pose);  
    target_crts_pose.position.x -= 0.2;
    waypoints.push_back(target_crts_pose);  
    target_crts_pose.position.z += 0.2;
    waypoints.push_back(target_crts_pose);  
    target_crts_pose.position.x -= 0.2;
    waypoints.push_back(target_crts_pose);  
    plan_cartesian_space(move_group, visual_tools, waypoints);

}
 
 
