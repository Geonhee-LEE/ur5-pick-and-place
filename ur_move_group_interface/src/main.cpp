/*
  *  UR5 move group interface

 *
 *  Created on: 2019. 05. 01.
 *      Author: Geonhee-LEE
 */


#include "ur_move_group_interface.h"

int main(int argc, char **argv)
{
 
    ros::init(argc, argv, "ur_move_group_interface");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    URMoveGroup ur;        
    
 
    ros::shutdown();
    return 0;
}
