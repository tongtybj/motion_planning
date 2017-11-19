// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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

#ifndef AERIAL_TRANSPORTATION_H
#define AERIAL_TRANSPORTATION_H

/* ros */
#include <ros/ros.h>

/* to get the motion planning method */
#include <pluginlib/class_loader.h>
#include <aerial_transportation/grasp_motion/base.h>

/* ros msg */
#include <aerial_robot_base/FlightNav.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/LinearMath/Transform.h> /* for vector3 */
#include <tf/transform_datatypes.h>
#include <std_srvs/SetBool.h>

namespace phase
{
  /* control mode */
  enum transportation_phase
    {
      IDLE,
      APPROACH,
      GRASPING,
      GRASPED,
      TRANSPORT,
      DROPPING,
      RETURN,
    };
};

namespace grasp_motion
{
  class Base;
};

class AerialTransportation
{
public:
  AerialTransportation(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~AerialTransportation(){}

  ros::Publisher uav_nav_pub_;

  bool positionConvergence(double thresh = 0);
  bool yawConvergence();

  /* not good, but can be easy to access from grasp motion planner */
  double approach_pos_threshold_; // the pos convergence posecondition for object approach
  double approach_yaw_threshold_; // the yaw convergence condition for object approach
  double func_loop_rate_;

  double approach_count_; //the convergence duration (sec)
  bool object_head_direction_; //whether need to consider the head direction of object
  double falling_speed_; //the vel to fall down to object
  double grasping_height_offset_; //the offset between the bottom of uav and the top plat of object
  double ascending_speed_; //the vel to carry up to object
  double transportation_threshold_; // the convergence condition to carry to box
  double transportation_count_; // the convergence duration
  double dropping_offset_;  //the offset between the top of box and the bottom of object

  /* base variable */
  int phase_;
  bool uav_odom_;
  bool object_found_;
  tf::Vector3 uav_cog_pos_;
  tf::Vector3 uav_cog_2d_pos_;
  tf::Vector3 uav_init_cog_2d_pos_;
  tf::Vector3 uav_target_cog_2d_pos_;
  tf::Vector3 uav_cog_vel_;
  double uav_cog_yaw_;
  double uav_target_cog_yaw_;
  double uav_target_height_;

  tf::Vector3 object_2d_pos_;
  double object_yaw_;

  /* config of target object */
  tf::Vector3 object_offset_; //[x,y,psi], the offset from the COG of object to the position control point of uav with the respect to world frame
  double object_height_; // in cheat mode, this is obtained by rosparam

  /* config of recycle box */
  tf::Vector3 box_pos_; // in cheat mode, this is obtained by rosparam
  tf::Vector3 box_offset_; // in cheat mode, this is obtained by rosparam

  tf::Transform getUavCogTransform()
  {
    return tf::Transform(tf::Matrix3x3(tf::createQuaternionFromYaw(uav_cog_yaw_)), uav_cog_2d_pos_);
  }

  tf::Transform getObjectTransform()
  {
    return tf::Transform(tf::Matrix3x3(tf::createQuaternionFromYaw(object_yaw_)), object_2d_pos_);
  }

private:
  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  /* grasp motion */
  boost::shared_ptr<grasp_motion::Base> grasp_motion_planner_;
  boost::shared_ptr< pluginlib::ClassLoader<grasp_motion::Base> > grasp_motion_planner_loader_ptr_;

  /* main thread */
  ros::Timer  func_timer_;

  /* ros publisher & subscirberrber */
  ros::ServiceServer set_motion_trigger_srv_;
  ros::Subscriber uav_odom_sub_;
  ros::Subscriber object_pos_sub_;

  /* rosparam based variables */
  bool verbose_;

  /* base function */
  void mainFunc(const ros::TimerEvent & e);

  void rosParamInit();

  void odomCallback(const nav_msgs::OdometryConstPtr & msg);
  void objectPoseCallback(const geometry_msgs::Pose2DConstPtr & object_msg);
  bool setMotionTrigger(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
};


#endif  // AERIAL_TRANSPORTATION_H
