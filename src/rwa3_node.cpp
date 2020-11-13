// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <algorithm>
#include <vector>
#include <thread>
#include <chrono>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <tf2/LinearMath/Quaternion.h>


void blackoutDetector(Competition * comp){
    ROS_INFO_STREAM("[blackoutDetector] started");
    while(comp->run_blackout_detection){
        if(comp->blackout_breakbeam){
            comp->blackout_breakbeam = false;
            comp->blackout = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }else{
            comp->blackout = true;
        }
    }
    ROS_INFO_STREAM("[blackoutDetector] ended");
}

bool isBlackout(Competition &comp){
    return comp.blackout;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rwa3_node");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    Competition comp(node);
    comp.init();

    std::string c_state = comp.getCompetitionState();
    comp.getClock();

    GantryControl gantry(node);
    gantry.init();
    gantry.goToPresetLocation(gantry.start_);

    //--1-Read order
    //--2-Look for parts in this order
    //--We go to this bin because a camera above
    //--this bin found one of the parts in the order
    gantry.goToPresetLocation(gantry.bin3_);


    //--You should receive the following information from a camera
    part my_part;
    my_part.type = "pulley_part_red";
    my_part.pose.position.x = 4.365789;
    my_part.pose.position.y = 1.173381;
    my_part.pose.position.z = 0.728011;
    my_part.pose.orientation.x = 0.012;
    my_part.pose.orientation.y = -0.004;
    my_part.pose.orientation.z = 0.002;
    my_part.pose.orientation.w = 1.000;

    //--get pose of part in tray from /ariac/orders
    part part_in_tray;
    part_in_tray.type = "pulley_part_red";
    part_in_tray.pose.position.x = -0.12;
    part_in_tray.pose.position.x = -0.2;
    part_in_tray.pose.position.x = 0.0;
    part_in_tray.pose.orientation.x = 0.0;
    part_in_tray.pose.orientation.y = 0.0;
    part_in_tray.pose.orientation.z = 0.0;
    part_in_tray.pose.orientation.w = 1.0;

    std::thread thread(blackoutDetector, &comp);//start blackout detector 
    for(int i = 0; i < 5; i++){
        comp.blackout = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        ROS_INFO_STREAM("Blackout: "<< isBlackout(comp));
    }
    //--Go pick the part
    gantry.pickPart(my_part);
    //--Go place the part
    gantry.placePart(part_in_tray, "agv2");
    
    while(isBlackout(comp)){
        ROS_INFO_STREAM("Blackout: "<< isBlackout(comp));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    
    ROS_INFO_STREAM("Sim done");
    comp.run_blackout_detection = false;
    thread.join();

    comp.endCompetition();
    spinner.stop();
    ros::shutdown();
    return 0;
}
