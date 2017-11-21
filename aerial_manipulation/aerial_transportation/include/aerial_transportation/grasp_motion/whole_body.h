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
  enum grasp_phase
    {
      IDLE,
      TRANSFORM,
      DESCEND,
      CONTACT,
      ROLL,
      GRASP,
    };

namespace roll_motion
{
  enum phase
    {
      IDLE,
      BASELINK_ROLL,
      FULL_CONTACT,
      NEIGHBOUR_ROLL,
    };
};

  class JointHandle
  {
  public:
    JointHandle(ros::NodeHandle nh, ros::NodeHandle nhp, double grasp_angle, int id):
      grasp_angle_(grasp_angle), target_angle_(grasp_angle), id_(id)
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
      nhp.param("approach_delta_angle", approach_delta_angle_, -0.45); // about 25deg
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", approach_delta_angle: " << approach_delta_angle_ <<std::endl;
      nhp.param("rough_grasping_delta_angle", rough_grasping_delta_angle_, 0.015);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", rough_grasping_delta_angle: " << rough_grasping_delta_angle_ <<std::endl;
      nhp.param("torque_grasping_delta_angle", torque_grasping_delta_angle_, 0.015);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", torque_grasping_delta_angle: " << torque_grasping_delta_angle_ <<std::endl;
      nhp.param("torque_grasp_threshold", torque_grasp_threshold_, 0.1);
      if(verbose) std::cout << "[hydrus grasp motion] ns: " << ns << ", torque_grasp_threshold: " << torque_grasp_threshold_ <<std::endl;

      approach_angle_ = grasp_angle_ + approach_delta_angle_;
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
                  target_angle_ += torque_grasping_delta_angle_;
                  modification_start_time_ = ros::Time::now().toSec();
                  grasping_start_time_ = ros::Time::now().toSec();
                  return false;
                }
          else if(current_torque_ > target_torque + torque_grasp_threshold_  &&
             ros::Time::now().toSec() - modification_start_time_ > angle_control_idle_duration_)
            {
              target_angle_ += torque_grasping_delta_angle_;
              modification_start_time_ = ros::Time::now().toSec();
              grasping_start_time_ = ros::Time::now().toSec();
              return false;
            }
          else
            {
              if(ros::Time::now().toSec() - grasping_start_time_ > grasping_duration_)
                return true;
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

    void approaching() { target_angle_ = approach_angle_; }

    bool convergence()
    {
      /* hard coding: 3deg check */
      if(fabs(target_angle_ - current_angle_) < 0.05) return true;
      else return false;
    }
    inline const double getTargetAngle() const {return target_angle_;}

    int id_;
    double grasping_start_time_; //the start time to grasp object
    double overload_start_time_; //the start time to modification the grasp
    double modification_start_time_; //the start time to modification the grasp

    double target_angle_;
    double current_angle_;
    double grasp_angle_;
    double approach_angle_; //the approach angle to the object
    double grasp_delta_angle_;
    double current_torque_;
    //int holding_rotation_direction_; //the rotation direction for grasping
    bool moving_;
    double temperature_; //reserve
    int error_; //reserve

    /* rosparam */
    double approach_delta_angle_; //the incerease angle to approach the object
    double rough_grasping_delta_angle_; //the grasping angle to grasp
    double torque_grasping_delta_angle_;  //the tighten angle to force-closure the object
    double angle_control_idle_duration_; // idle duration to control joint angle for tighten/release
    double overload_check_duration_; // idle duration to control joint angle for tighten/release
    double torque_grasp_threshold_;
    double grasping_duration_; //the hold ok time count
    double grasp_min_torque_; // guarantee force-closure
    double grasp_max_torque_; // avoid overload

    bool rolling_motion_test_;
  };

  class WholeBody :public grasp_motion::Base
  {
  public:
    WholeBody();
    ~WholeBody(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, AerialTransportation* transportator) override;


    static constexpr uint8_t OVERLOAD_FLAG = 0x20;

    /* overwrite */
    tf::Vector3 getUavTargetApproach2DPos(tf::Vector3 object_2d_pos, double objecy_yaw) override;
    double getUavTargetApproachYaw(double object_yaw) override;
    bool grasp() override;
    bool drop() override;

    bool rollMotion();

  private:
    /* ros publisher & subscirber */
    ros::Publisher joint_ctrl_pub_;
    ros::Publisher  flight_config_pub_;
    ros::Subscriber joint_states_sub_;
    ros::Subscriber joint_motors_sub_;
    ros::ServiceClient control_gain_client_;

    /* plugin for grasp searching */
    boost::shared_ptr<grasp_form_search::GraspFormSearch> grasp_form_search_method_;
    /* uav kinematics */
    boost::shared_ptr<TransformController> uav_kinematics_;

    /* base variable */
    bool verbose_;
    bool debug_verbose_;
    int phase_; // sub pahse for GRASPPING_PAHSE
    std::vector<grasp_motion::JointHandle> joints_;

    int baselink_;
    double approach_offset_dist_; //the distance between uav and object in the approaching phase.
    double pose_fixed_count_; //the convergence duration (sec)
    double grasping_height_threshold_; //the height condition to grasp to object

    double approach_acc_tilt_angle_;
    double contact_tilt_angle_;
    double contact_vel_threshold_;
    double rolling_angle_threshold_;
    double rolling_yaw_threshold_;

    double default_yaw_p_gain_;
    double rolling_yaw_p_gain_;

    /* base function */
    tf::Vector3 getBaseLinkRotorOrigin()
    {
      tf::Vector3 cog2baselink_rotor;
      tf::vectorEigenToTF(uav_kinematics_->getRotorsOriginFromCog().at(baselink_), cog2baselink_rotor);
      return transportator_->getUavCogTransform() * cog2baselink_rotor;
    }

    void rosParamInit() override;
    void jointControlParamInit();
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states_msg); //get calibrated joints angle vector
    void jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg); //get the torque load nad temprature from each joint

    void sendControlGain(double gain);
  };
};
#endif
