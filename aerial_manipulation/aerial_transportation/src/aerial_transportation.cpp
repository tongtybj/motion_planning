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

#include <aerial_transportation/aerial_transportation.h>

namespace
{
  int cnt = 0;
}

AerialTransportation::AerialTransportation(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp),
  uav_odom_(false), object_found_(false),
  uav_target_height_(0)
{
  rosParamInit();

  /* grasping motion pluginlib */
  try
    {
      grasp_motion_planner_loader_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<grasp_motion::Base> >( new pluginlib::ClassLoader<grasp_motion::Base>("aerial_transportation", "grasp_motion::Base"));
      std::string motion_planner_plugin_name;
      std::string plugin_name;
      nhp_.param ("grasp_motion_planner_plugin_name", plugin_name, std::string("grasp_motion/whole_body"));
      if(verbose_) std::cout << "[aerial transportation] grasp_motion_planner_plugin_name: " << plugin_name << std::endl;
      grasp_motion_planner_ = grasp_motion_planner_loader_ptr_->createInstance(plugin_name);
      grasp_motion_planner_->initialize(nh_, nhp_, this);
    }
  catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

  /* pub & sub */
  std::string topic_name;
  nhp_.param("uav_nav_pub_name", topic_name, std::string("/uav/nav"));
  uav_nav_pub_ = nh_.advertise<aerial_robot_base::FlightNav>(topic_name, 1);
  nhp_.param("uav_odom_sub_name", topic_name, std::string("/uav/odom"));
  uav_odom_sub_ = nh_.subscribe(topic_name, 1, &AerialTransportation::odomCallback, this);
  nhp_.param("object_pos_sub_name", topic_name, std::string("/object"));
  object_pos_sub_ = nh_.subscribe(topic_name, 1, &AerialTransportation::objectPoseCallback, this);
  nhp_.param("set_motion_trigger_srv_name", topic_name, std::string("/start_aerial_transportation"));
  set_motion_trigger_srv_ = nh_.advertiseService(topic_name, &AerialTransportation::setMotionTrigger, this);

  /* timer init */
  nhp_.param("motion_func_loop_rate", func_loop_rate_, 40.0);
  func_timer_ = nhp_.createTimer(ros::Duration(1.0 / func_loop_rate_), &AerialTransportation::mainFunc,this);

}

void AerialTransportation::rosParamInit()
{
  std::string ns = nhp_.getNamespace();

  nhp_.param("verbose", verbose_, false);
  nhp_.param("init_phase", phase_, 0); //IDLE
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", init_phase: " << phase_ <<std::endl;
  nhp_.param("approach_pos_threshold", approach_pos_threshold_, 0.05);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", approach_pos_threshold: " << approach_pos_threshold_ <<std::endl;
  nhp_.param("approach_yaw_threshold", approach_yaw_threshold_, 0.1);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", approach_yaw_threshold: " << approach_yaw_threshold_ <<std::endl;
  nhp_.param("approach_count", approach_count_, 2.0); //sec
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", approach_count: " << approach_count_ <<std::endl;
  nhp_.param("object_head_direction", object_head_direction_, false);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", object_head_direction: " << object_head_direction_ <<std::endl;
  nhp_.param("falling_speed", falling_speed_, -0.04);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", falling_speed: " << falling_speed_ <<std::endl;
  nhp_.param("ascending_speedw", ascending_speed_, 0.1);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", ascending_speed: " << ascending_speed_ <<std::endl;
  nhp_.param("transportation_threshold", transportation_threshold_, 0.1);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", transportation_threshold: " << transportation_threshold_ <<std::endl;
  nhp_.param("transportation_count", transportation_count_, 2.0); //sec
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", transportation_count: " << transportation_count_ <<std::endl;
  nhp_.param("dropping_offset", dropping_offset_, 0.15);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", dropping_offset: " << dropping_offset_ <<std::endl;

  /* no recognition */
  nhp_.param("grasping_height", grasping_height_, 0.2);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", grasping_height: " << grasping_height_ <<std::endl;
  /* recycle box config */
  nhp_.param("box_x", box_pos_.m_floats[0], 1.145);
  nhp_.param("box_y", box_pos_.m_floats[1], 0.02);
  nhp_.param("box_z", box_pos_.m_floats[2], 0.2);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", box_pos: [" << box_pos_.x() << ", " <<  box_pos_.y()  << ", "<<  box_pos_.z() << "]" <<std::endl;
  nhp_.param("box_offset_x", box_offset_.m_floats[0], 0.0);
  nhp_.param("box_offset_y", box_offset_.m_floats[1], 0.0);
  nhp_.param("box_offset_z", box_offset_.m_floats[2], 0.0);
  if(verbose_) std::cout << "[aerial transportation] ns: " << ns << ", box_offset: [" << box_offset_.x() <<   ", "<< box_offset_.y() <<  ", "<<  box_offset_.z() << "]" <<std::endl;

}

bool AerialTransportation::setMotionTrigger(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  if(!uav_odom_)
    {
      res.success = false;
      res.message = std::string("no uav odom received");
      ROS_WARN("[aerial transportation], can not start motion, since %s", res.message.c_str());
      return true;
    }

  if(!object_found_)
    {
      res.success = false;
      res.message = std::string("no object position received");
      ROS_WARN("[aerial transportation], can not start motion, since %s", res.message.c_str());
      return true;
    }

  if(req.data)
    {
      if(phase_ !=  phase::IDLE)
        {
          res.success = false;
          res.message = std::string("phase is not idle");
          ROS_WARN("[aerial transportation], can not start motion, since %s", res.message.c_str());
          return true;
        }

      phase_ ++;
      uav_init_cog_2d_pos_ = uav_cog_2d_pos_;

      /* get target position only once */
      uav_target_cog_2d_pos_ = grasp_motion_planner_->getUavTargetApproach2DPos();
      uav_target_cog_yaw_ = grasp_motion_planner_->getUavTargetApproachYaw();
      /* nomalized yaw */
      if(uav_target_cog_yaw_ > M_PI)  uav_target_cog_yaw_ -= (2 * M_PI);
      else if(uav_target_cog_yaw_ < -M_PI)  uav_target_cog_yaw_ += (2 * M_PI);

      ROS_INFO("[aerial transportation] shift to APPROACH, uav_target_cog_2d_pos: [%f, %f], uav_target_cog_yaw: %f", uav_target_cog_2d_pos_.x(), uav_target_cog_2d_pos_.y(), uav_target_cog_yaw_);

      res.success = false;
      res.message = std::string("start motion");

      return true;
    }

  return false;
}

/* get uav CoG odometry */
void AerialTransportation::odomCallback(const nav_msgs::OdometryConstPtr & msg)
{
  if(!uav_odom_) uav_odom_ = true;

  tf::pointMsgToTF(msg->pose.pose.position, uav_cog_pos_);
  tf::vector3MsgToTF(msg->twist.twist.linear, uav_cog_vel_);
  uav_cog_2d_pos_.setValue(uav_cog_pos_.x(), uav_cog_pos_.y(), 0);

  uav_cog_yaw_ = tf::getYaw(msg->pose.pose.orientation);
}

/* get object 2D pose */
void AerialTransportation::objectPoseCallback(const geometry_msgs::Vector3StampedConstPtr & object_msg)
{
  if(!object_found_) object_found_ = true;

  object_2d_pos_.setValue(object_msg->vector.x, object_msg->vector.y, 0);
  object_yaw_ = object_msg->vector.z;

}

void AerialTransportation::mainFunc(const ros::TimerEvent & e)
{
  switch(phase_)
    {
    case phase::APPROACH:
      {
        assert(uav_odom_ && object_found_);

        /* send flight nav */
        aerial_robot_base::FlightNav nav_msg;
        nav_msg.header.stamp = ros::Time::now();
        nav_msg.target = aerial_robot_base::FlightNav::COG;
        nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
        nav_msg.target_pos_x = uav_target_cog_2d_pos_.x();
        nav_msg.target_pos_y = uav_target_cog_2d_pos_.y();
        nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;

        if(object_head_direction_)
          {
            nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
            nav_msg.target_psi = uav_target_cog_yaw_;
          }
        uav_nav_pub_.publish(nav_msg);

        /* phase shift condition */
        if(positionConvergence() && yawConvergence())
          {
            if(++cnt > (approach_count_ * func_loop_rate_))
              {
                ROS_WARN("[aerial transportation] Succeed to approach object, shift to GRASPING");
                phase_++;
                cnt = 0; // convergence reset
                uav_target_height_ = uav_cog_pos_.z();
              }
          }
        break;
      }
    case phase::GRASPING:
      {
        if(grasp_motion_planner_->grasp())
          {
            ROS_INFO("[aerial transportation] Scueed to grasp object");
            phase_++;
            cnt = 0;
            uav_target_height_ = uav_cog_pos_.z();
          }
        break;
      }
    case phase::GRASPED:
      {
        /* height calc part */
        uav_target_height_ += (ascending_speed_ / func_loop_rate_);
        if(uav_target_height_ > box_pos_.z() + dropping_offset_)
          uav_target_height_ = box_pos_.z() + dropping_offset_;

        /* send nav msg */
        aerial_robot_base::FlightNav nav_msg;
        nav_msg.header.stamp = ros::Time::now();
        nav_msg.target = aerial_robot_base::FlightNav::COG;
        nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
        nav_msg.target_pos_x = uav_cog_2d_pos_.x();
        nav_msg.target_pos_y = uav_cog_2d_pos_.y();
        nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
        nav_msg.target_pos_z = uav_target_height_;
        nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        uav_nav_pub_.publish(nav_msg);

        if(fabs(box_pos_.z() + dropping_offset_ - uav_cog_pos_.z())  < 0.05) //0.05m, hard-coding
          {
            ROS_INFO("[aerial transportation] Shift to TRANSPORT");
            phase_++;
            cnt = 0;
          }
        break;
      }
    case phase::TRANSPORT:
      {
        uav_target_cog_2d_pos_ = box_pos_ + box_offset_;
        uav_target_cog_2d_pos_.setZ(0);
        /* nav part */
        aerial_robot_base::FlightNav nav_msg;
        nav_msg.header.stamp = ros::Time::now();
        nav_msg.target = aerial_robot_base::FlightNav::COG;
        nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
        nav_msg.target_pos_x = uav_target_cog_2d_pos_.x();
        nav_msg.target_pos_y = uav_target_cog_2d_pos_.y();
        nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        uav_nav_pub_.publish(nav_msg);

        /* phase shift condition */
        if(positionConvergence(transportation_threshold_))
          {
            if(++cnt > (transportation_count_ * func_loop_rate_))
              {
                ROS_INFO("[aerial transportation] Succeed to approach to box, and drop!!");
                phase_ ++;
                cnt = 0;
              }
          }
        break;
      }
    case phase::DROPPING:
      {
        if(grasp_motion_planner_->drop())
          {
            ROS_INFO("[aerial transportation] Finish dropping");
            phase_++;
            cnt = 0;
          }
        break;
      }
    case phase::RETURN:
      {
        aerial_robot_base::FlightNav nav_msg;
        nav_msg.header.stamp = ros::Time::now();
        nav_msg.target = aerial_robot_base::FlightNav::COG;
        nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
        nav_msg.target_pos_x = uav_init_cog_2d_pos_.x();
        nav_msg.target_pos_y = uav_init_cog_2d_pos_.y();
        nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::NO_NAVIGATION;
        uav_nav_pub_.publish(nav_msg);

        phase_ = phase::IDLE;
        ROS_INFO("Return, and set phase to IDLE");
        break;
      }
    default:
      {
        cnt = 0;
        //ROS_INFO("idle");
        break;
      }
    }
}

bool AerialTransportation::positionConvergence(double thresh)
{
  if(thresh == 0) thresh = approach_pos_threshold_;

  //ROS_INFO("thresh: %f, uav_target_cog_2d_pos_: [%f, %f, %f] - uav_cog_2d_pos_: [%f, %f, %f]", thresh, uav_target_cog_2d_pos_.x(), uav_target_cog_2d_pos_.y(), uav_target_cog_2d_pos_.z(), uav_cog_2d_pos_.x(), uav_cog_2d_pos_.y(), uav_cog_2d_pos_.z());

  return ((uav_target_cog_2d_pos_ - uav_cog_2d_pos_).length() <  thresh)?true:false;
}

bool AerialTransportation::yawConvergence()
{

  //ROS_INFO("uav_target_cog_yaw_: %f, uav_cog_yaw_: %f, approach_yaw_threshold_: %f", uav_target_cog_yaw_, uav_cog_yaw_, approach_yaw_threshold_);
  if(object_head_direction_)
    return (fabs(uav_target_cog_yaw_ - uav_cog_yaw_) < approach_yaw_threshold_)?true:false;
  else
    return true;
}
