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

#ifndef GRASP_MOTION_WHOLE_BODY_PLUGIN_H
#define GRASP_MOTION_WHOLE_BODY_PLUGIN_H

/* note: we choose the link which has the same orientation with CoG frame, that is the baselink */

#include <aerial_transportation/grasp_motion/base.h> // plugin base class
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>
#include <hydrus/AddExtraModule.h>
#include <aerial_robot_base/FlightConfigCmd.h>
#include <vector>
#include <string>


/* grasp form searching node */
#include <aerial_transportation/grasp_form_searching/grasp_form_search.h>

/* set control gain: dynamic reconfigure by rosserive */
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

namespace grasp_motion
{
  enum grasp
    {
      IDLE,
      TRANSFORM,
      DESCEND,
      CONTACT,
      GRASP,

    };
namespace contact_status
{
  enum phase
    {
      NO,
      BASELINK,
      NEIGHBOUR,
      BOTH,
    };
};

  class JointHandle
  {
  public:
    JointHandle(ros::NodeHandle nh, ros::NodeHandle nhp, int id):
      id_(id), target_angle_(0), current_angle_(0), current_torque_(0), init_flag_(false)
    {
      bool verbose;
      std::string ns = nhp.getNamespace();
      nhp.param("verbose", verbose, false);
      nhp.param("grasp_min_torque", grasp_min_torque_, 0.2); //20%
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", grasp_min_torque: " << grasp_min_torque_ <<std::endl;
      nhp.param("grasp_max_torque", grasp_max_torque_, 0.5); //50%
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", grasp_max_torque: " << grasp_max_torque_ <<std::endl;
      nhp.param("grasping_duration", grasping_duration_, 1.0);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", grasping_duration: " << grasping_duration_ <<std::endl;
      nhp.param("angle_control_idle_duration", angle_control_idle_duration_, 0.5);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", angle_control_idle_duration: " << angle_control_idle_duration_ <<std::endl;
      nhp.param("overload_check_duration", overload_check_duration_, 0.5);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", overload_check_duration: " << overload_check_duration_ <<std::endl;
      nhp.param("rough_grasping_delta_angle", rough_grasping_delta_angle_, 0.015);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", rough_grasping_delta_angle: " << rough_grasping_delta_angle_ <<std::endl;
      nhp.param("torque_grasping_delta_angle", torque_grasping_delta_angle_, 0.015);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", torque_grasping_delta_angle: " << torque_grasping_delta_angle_ <<std::endl;
      nhp.param("torque_grasp_threshold", torque_grasp_threshold_, 0.1);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", torque_grasp_threshold: " << torque_grasp_threshold_ <<std::endl;
    }
    ~JointHandle(){}

    bool grasping(double target_torque = 0, bool reset = false)
    {
      if(reset)
        {
          grasping_start_time_ = ros::Time::now().toSec();
          overload_start_time_ = ros::Time::now().toSec();
          modification_start_time_ = ros::Time::now().toSec();
        }

      if(target_torque == 0)
        {
          //ROS_INFO("current_torque_: %f, grasp_min_torque_: %f", current_torque_, grasp_min_torque_);
          if(current_torque_ < grasp_min_torque_)
            {
              target_angle_ += rough_grasping_delta_angle_;
              grasping_start_time_ = ros::Time::now().toSec();
              return false;
            }
          else
            {
              if(ros::Time::now().toSec() - grasping_start_time_ > grasping_duration_)
                return true;
            }
        }
      else
        {
          /* simple torque control mode */
          if(current_torque_ < target_torque -  torque_grasp_threshold_  &&
             ros::Time::now().toSec() - modification_start_time_ > angle_control_idle_duration_)
                {
                  //ROS_INFO("lack"); //nenetti
                  target_angle_ += torque_grasping_delta_angle_;
                  modification_start_time_ = ros::Time::now().toSec();
                  grasping_start_time_ = ros::Time::now().toSec();
                  return false;
                }
          else if(current_torque_ > target_torque + torque_grasp_threshold_  &&
             ros::Time::now().toSec() - modification_start_time_ > angle_control_idle_duration_)
            {
              //ROS_INFO("exceed"); //nenetti
              target_angle_ -= torque_grasping_delta_angle_;
              modification_start_time_ = ros::Time::now().toSec();
              grasping_start_time_ = ros::Time::now().toSec();
              return false;
            }
          else
            {
              if(ros::Time::now().toSec() - grasping_start_time_ > grasping_duration_)
                {
                  //ROS_INFO("force-closure"); //nenetti
                  return true;
                }
            }
        }

      /* avoid overload */
      if(current_torque_ > grasp_max_torque_ && (ros::Time::now().toSec() - overload_start_time_ > overload_check_duration_))
        {
          overload_start_time_ = ros::Time::now().toSec();
          target_angle_ -=  torque_grasping_delta_angle_;
          ROS_WARN("[hydrus grasp motion]: overload in joint%d: %f; error code: %d", id_ + 1, current_torque_, error_);
        }
    }

    inline const double getTargetAngle() const {return target_angle_;}
    void setTargetAngle(double target_angle) {target_angle_ = target_angle;}

    bool convergence()
    {
      /* hard coding: 3deg check */
      if(fabs(target_angle_ - current_angle_) < 0.05) return true;
      else return false;
    }

    bool init_flag_;
    int id_;
    double grasping_start_time_; //the start time to grasp object
    double overload_start_time_; //the start time to modification the grasp
    double modification_start_time_; //the start time to modification the grasp

    double target_angle_;
    double current_angle_;
    double current_torque_;
    //int holding_rotation_direction_; //the rotation direction for grasping
    bool moving_;
    double temperature_; //reserve
    int error_; //reserve

    /* rosparam */
    double rough_grasping_delta_angle_; //the grasping angle to grasp
    double torque_grasping_delta_angle_;  //the tighten angle to force-closure the object
    double angle_control_idle_duration_; // idle duration to control joint angle for tighten/release
    double overload_check_duration_; // idle duration to control joint angle for tighten/release
    double torque_grasp_threshold_;
    double grasping_duration_; //the hold ok time count
    double grasp_min_torque_; // guarantee force-closure
    double grasp_max_torque_; // avoid overload
  };

  class WholeBody :public grasp_motion::Base
  {
  public:
    WholeBody();
    ~WholeBody(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, AerialTransportation* transportator) override;

    static constexpr uint8_t OVERLOAD_FLAG = 0x20;

    /* overwrite */
    tf::Vector3 getUavTargetApproach2DPos() override;
    double getUavTargetApproachYaw() override;
    bool grasp() override;
    bool drop() override;

  private:
    /* ros publisher & subscirber */
    ros::Publisher joint_ctrl_pub_;
    ros::Publisher  flight_config_pub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber joint_motors_sub_;
    ros::ServiceClient control_gain_client_;

    /* debug */
    ros::Publisher joint_motors_pub_;
    ros::Publisher contact_state_pub_;

    /* plugin for grasp searching */
    boost::shared_ptr<grasp_form_search::GraspFormSearch> grasp_form_search_method_;
    /* uav kinematics */
    boost::shared_ptr<TransformController> uav_kinematics_;

    /* base variable */
    bool verbose_;
    bool debug_verbose_;
    std::vector<grasp_motion::JointHandle> joints_;

    int baselink_;
    double approach_delta_angle_; //the incerease angle to approach the object
    double approach_offset_dist_; //the distance between uav and object in the approaching phase.
    double pose_fixed_count_; //the convergence duration (sec)
    double full_contact_count_; //the convergence duration (sec)
    double grasping_height_threshold_; //the height condition to grasp to object

    double contact_tilt_angle_;
    double contact_dist_threshold_;
    double contact_rolling_yaw_angle_;
    double default_yaw_p_gain_;
    double contact_yaw_p_gain_;

    /* base function */
    tf::Vector3 getRotorOriginInWordFrame(int index)
    {
      tf::Vector3 cog2rotor;
      tf::vectorEigenToTF(uav_kinematics_->getRotorsOriginFromCog().at(index), cog2rotor);
      return transportator_->getUavCog2dTransform() * cog2rotor;
    }

    tf::Transform getContactFrame(int index)
    {
      tf::Vector3 contact_p;
      tf::vectorEigenToTF(grasp_form_search_method_->getBestContactP().at(index), contact_p);

      tf::Quaternion contact_rot;
      tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(index), contact_rot); //cylinder: rot = E
      //ROS_INFO("contact_p%d  in object frame: [%f, %f, %f], yaw: %f", index + 1, contact_p.x(), contact.y(), contact_p.z(), tf::getYaw(contact_rot));

      return tf::Transform(contact_rot, contact_p);
    }

    void rosParamInit() override;
    void jointControlParamInit();
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states_msg); //get calibrated joints angle vector
    void jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg); //get the torque load nad temprature from each joint

    void sendControlGain(double gain);

    void getContactStatus(int& contact_status, double& baselink_dist, double& neighbour_dist);
  };
};
#endif
