/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <go_forward_recovery/go_forward_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_DECLARE_CLASS(go_forward_recovery, GoForwardRecovery, go_forward_recovery::GoForwardRecovery, nav_core::RecoveryBehavior)

namespace go_forward_recovery {
GoForwardRecovery::GoForwardRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void GoForwardRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

GoForwardRecovery::~GoForwardRecovery(){
  delete world_model_;
}

double GoForwardRecovery::null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    ROS_WARN("RANGE OVER");
  }
  return target;
}

void GoForwardRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  ROS_INFO("center: [%lf]", msg->ranges[(-msg->angle_min)/msg->angle_increment]);
  double center_number = (-msg->angle_min)/msg->angle_increment;
  double center = msg->ranges[center_number];
  double left = msg->ranges[center_number+255];
  double right = msg->ranges[center_number-255];

  center = null_check(center);
  left = null_check(left);
  right = null_check(right);

  ROS_INFO("center: [%lf], left: [%lf], right: [%lf]", center, left, right);
  //ROS_INFO("center number: [%lf]", (-msg->angle_min)/msg->angle_increment);

  if(center < 0.5){
    ROS_WARN("center warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;
  }
  if(left < 0.5){
    ROS_WARN("left warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = -1.0;
  }
  if(right < 0.5){
    ROS_WARN("right warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;
  }
  if(center >=0.5 && left >= 0.5 && right >= 0.5){
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  ROS_INFO("x: %lf, y: %lf, z: %lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  vel_pub.publish(cmd_vel);
}

void GoForwardRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the GoForwardRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Go forward recovery behavior started.");

  ros::Rate r(frequency_);
  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

  scan_sub = n.subscribe("scan", 1000, &GoForwardRecovery::scanCallback, this);
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  //double current_angle = -1.0 * M_PI;

  //bool got_180 = false;

  //double start_offset = 0 - angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
  //while(n.ok()){
    //local_costmap_->getRobotPose(global_pose);

    //double norm_angle = angles::normalize_angle(tf::getYaw(global_pose.getRotation()));
    //current_angle = angles::normalize_angle(norm_angle + start_offset);

    //compute the distance left to rotate
    //double dist_left = M_PI - current_angle;

    //double x = global_pose.getOrigin().x(), y = global_pose.getOrigin().y();

    //check if that velocity is legal by forward simulating
    //double sim_angle = 0.0;
    //while(sim_angle < dist_left){
      //double theta = tf::getYaw(global_pose.getRotation()) + sim_angle;

      //make sure that the point is legal, if it isn't... we'll abort
      //double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      //if(footprint_cost < 0.0){
      //  ROS_ERROR("Go forward recovery can't go forward in place because there is a potential collision. Cost: %.2f", footprint_cost);
      //  return;
      //}

      //sim_angle += sim_granularity_;
    //}

    //compute the velocity that will let us stop by the time we reach the goal
    //double vel = sqrt(2 * acc_lim_th_ * dist_left);

    //make sure that this velocity falls within the specified limits
    //vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

    //cmd_vel.linear.x = 0.2;
    //cmd_vel.linear.y = 0.0;
    //cmd_vel.angular.z = 0.0;

    //vel_pub.publish(cmd_vel);
    //ROS_WARN("cmd_vel published.");

    //makes sure that we won't decide we're done right after we start
    //if(current_angle < 0.0)
    //  got_180 = true;

    //if we're done with our in-place rotation... then return
    //if(got_180 && current_angle >= (0.0 - tolerance_))
    //  return;

    //r.sleep();
  //}
}
};
