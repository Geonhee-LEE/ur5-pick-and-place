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
};



#endif
