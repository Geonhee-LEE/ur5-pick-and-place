#ifndef UR_MOVE_GROUP_INTERFACE_H_
#define UR_MOVE_GROUP_INTERFACE_H_


// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//c++
#include <string>
#include <iostream>

 

 
 
class URMoveGroup
{
    public:
        URMoveGroup(void);
        ~URMoveGroup(void);
    private:
        ros::NodeHandle nh_;
 
    public:
        std::string PLANNING_GROUP;

        // The :move_group_interface:`MoveGroup` class can be easily
        // setup using just the name of the planning group you would like to control and plan for.
        //moveit::planning_interface::MoveGroupInterface move_group;

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to add and remove collision objects in our "virtual world" scene
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Raw pointers are frequently used to refer to the planning group for improved performance.
        const robot_state::JointModelGroup* joint_model_group;

        moveit_msgs::CollisionObject worktop_obj;
        std::vector<moveit_msgs::CollisionObject> collision_objects;

        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;

        std::string planning_frame;
        std::string endeffector_frame;

        // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
        Eigen::Affine3d _text_pose;

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan _plan;

        // Current state
        moveit::core::RobotStatePtr current_state;

        // Plan result; true or false
        bool success;

    public:
        void init_move_group(moveit::planning_interface::MoveGroupInterface& ); // Initialization of MoveIt group interface
        void init_visualization( moveit_visual_tools::MoveItVisualTools& );  // Initialization of visualization tools
        void start(moveit::planning_interface::MoveGroupInterface&, moveit_visual_tools::MoveItVisualTools&);   // start
        void create_worktop(std::string collision_id);

        void updateCurrentState(moveit::planning_interface::MoveGroupInterface&);
        std::vector<double> copyJointGroupPosition();

        void plan2goal(moveit::planning_interface::MoveGroupInterface&, geometry_msgs::Pose);
        void plan2goal(moveit::planning_interface::MoveGroupInterface&, moveit_visual_tools::MoveItVisualTools& , geometry_msgs::Pose);
        void plan4joint_space(moveit::planning_interface::MoveGroupInterface&, std::vector<double>);
        void plan4joint_space(moveit::planning_interface::MoveGroupInterface&, moveit_visual_tools::MoveItVisualTools&, std::vector<double>);
        void plan_with_path_constraint(moveit::planning_interface::MoveGroupInterface&, moveit_msgs::OrientationConstraint ocm , geometry_msgs::Pose , geometry_msgs::Pose );
        void plan_with_path_constraint(moveit::planning_interface::MoveGroupInterface&, moveit_visual_tools::MoveItVisualTools&, moveit_msgs::OrientationConstraint ocm, geometry_msgs::Pose , geometry_msgs::Pose );
 
    public:
 
};



#endif
