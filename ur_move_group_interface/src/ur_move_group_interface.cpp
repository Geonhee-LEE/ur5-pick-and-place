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


void URMoveGroup::plan2goal(MoveGroupInterface& move_group, geometry_msgs::Pose target_pose)
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

void URMoveGroup::plan2goal(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, geometry_msgs::Pose target_pose)
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

void URMoveGroup::plan4joint_space(MoveGroupInterface& move_group, vector<double> target_joint_group_positions)
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
        
void URMoveGroup::plan4joint_space(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools, vector<double> target_joint_group_positions)
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

void URMoveGroup::updateCurrentState(MoveGroupInterface& move_group)
{
    // Update current state global variable
    current_state = move_group.getCurrentState();
}

std::vector<double> URMoveGroup::copyJointGroupPosition()
{
    // Copy joint_model_group to the join group position 
    // target_joint_group_positions[0(base_link), ..., 5(wrist3)]
    std::vector<double> target_joint_group_positions;

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
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());

  start_state.setFromIK(joint_model_group, start_pose);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
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
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*move_group.getCurrentState());
  start_state.setFromIK(joint_model_group, start_pose);
  move_group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
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

void URMoveGroup::start(MoveGroupInterface& move_group, MoveItVisualTools& visual_tools)
{
    //Planning to a Pose goal
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.20;
    target_pose.position.y = -0.4;
    target_pose.position.z = 0.7;
    plan2goal(move_group, visual_tools, target_pose);

    //Planning to a joint-space goal
    updateCurrentState(move_group);
    std::vector<double> target_joint_group_positions = copyJointGroupPosition();
    target_joint_group_positions[0] = -1.0;  // radians
    plan4joint_space(move_group, visual_tools, target_joint_group_positions);

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
    start_pose.position.x = 0.55;
    start_pose.position.y = -0.05;
    start_pose.position.z = 0.5;
    plan_with_path_constraint(move_group, visual_tools, ocm, start_pose, target_pose);
}
 
 
