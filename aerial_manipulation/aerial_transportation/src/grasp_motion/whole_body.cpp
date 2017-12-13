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

#include <aerial_transportation/grasp_motion/whole_body.h>
#include <aerial_transportation/ObjectConfigure.h>

using namespace Eigen;

namespace grasp_motion
{

  namespace
  {
    bool once_flag = true;
    int cnt = 0;
    double G = 9.797;

    int grasp_phase = grasp_motion::IDLE;
    //int contact_status_ = cotact_status::NO;
    /* for convex polygon rolling */
    int neighbour_link = 0;
    int center_joint = 0;

    double duct_radius = 0;
    double object_radius = 0;
    int object_type = 0;

    int grasping_level = -1;
    VectorXd v_grasp_torque;
    static constexpr int CONVEX_POLYGONAL_COLUMN = aerial_transportation::ObjectConfigure::Request::CONVEX_POLYGONAL_COLUMN;
    static constexpr int CYLINDER = aerial_transportation::ObjectConfigure::Request::CYLINDER;
  }

  WholeBody::WholeBody() {}

  void WholeBody::initialize(ros::NodeHandle nh, ros::NodeHandle nhp, AerialTransportation* transportator)
  {
    /* super class init */
    grasp_motion::Base::initialize(nh, nhp, transportator);

    /* initialize for grasp form search method */
    grasp_form_search_method_ = boost::shared_ptr<grasp_form_search::GraspFormSearch>(new grasp_form_search::GraspFormSearch(nh, nhp));
    uav_kinematics_ = grasp_form_search_method_->getUavKinematics();

    /* check the grasp form search method whether get the result form file */
    assert(grasp_form_search_method_->getBestContactP().size() > 0);
    assert(uav_kinematics_->getRotorNum() > 0);
    assert(grasp_form_search_method_->getBestPhi().size() == uav_kinematics_->getRotorNum());

    /* ros param init */
    rosParamInit();

    /* set the joints info */
    joints_.clear();
    for(int i = 0; i < uav_kinematics_->getRotorNum() - 1; i++)
      joints_.push_back(grasp_motion::JointHandle(nh_, nhp_, i));

    /* init for the polygon roll motion */
    baselink_ = grasp_form_search_method_->getBaselink();
    duct_radius = grasp_form_search_method_->getDuctRadius();
    object_radius = grasp_form_search_method_->getObjectRadius();
    object_type = grasp_form_search_method_->getObjectType();
    if(uav_kinematics_->getRotorNum() / 2  > baselink_)
      {
        neighbour_link = baselink_ + 1;
        center_joint = baselink_;
      }
    else
      {
        neighbour_link = baselink_ - 1;
        center_joint = neighbour_link;
      }
    ROS_INFO("[hydrus grasp motion] polygon rolling motion, neighbour link: %d, center joint: %d", neighbour_link, center_joint);

    /* torque grasp */
    double torque_grasp_min_torque_rate;
    nhp_.param("torque_grasp_min_torque_rate", torque_grasp_min_torque_rate, 1.1); // [m]
    v_grasp_torque = grasp_form_search_method_->getBestTau();
    v_grasp_torque *= (joints_.at(0).grasp_min_torque_ * torque_grasp_min_torque_rate / v_grasp_torque.minCoeff());
    if(v_grasp_torque.maxCoeff() > joints_.at(0).grasp_max_torque_) ROS_FATAL("the grasp max torque %f exceed the valid value %f, the min is %f ", v_grasp_torque.maxCoeff(), joints_.at(0).grasp_max_torque_, v_grasp_torque.minCoeff());

    /* nenetti test */
    //grasp_phase = grasp_motion::GRASP;
    //transportator_->phase_ = phase::GRASPING;

    /* rqt_plot debug */
    grasp_phase = grasp_motion::CONTACT;
    transportator_->phase_ = phase::GRASPING;

    /* ros pub sub init */
    std::string topic_name;
    nhp_.param("joint_ctrl_pub_name", topic_name, std::string("/hydrus/joints_ctrl"));
    joint_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>(topic_name, 1);
    nhp_.param("joint_states_sub_name", topic_name, std::string("/joint_states"));
    joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>(topic_name, 1, &grasp_motion::WholeBody::jointStatesCallback, this);
    nhp_.param("joint_motors_sub_name", topic_name, std::string("/joint_motors"));
    joint_motors_sub_ = nh_.subscribe<dynamixel_msgs::MotorStateList>(topic_name, 1, &grasp_motion::WholeBody::jointMotorStatusCallback, this);
    nhp_.param("flight_config_cmd_pub_name", topic_name, std::string("/flight_config_cmd"));
    flight_config_pub_ = nh_.advertise<aerial_robot_base::FlightConfigCmd>(topic_name, 10);

    nhp_.param("control_gain_cmd_srv_name", topic_name, std::string("/hydrusx/set_parameters"));
    control_gain_client_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(topic_name);
    sendControlGain(default_yaw_p_gain_);

    /* debug */
    joint_motors_pub_ = nh_.advertise<dynamixel_msgs::MotorStateList>("/joint_motors/debug", 1);
    contact_state_pub_ = nh_.advertise<geometry_msgs::Vector3>("/contact_state/debug", 1);
  }

  void WholeBody::rosParamInit()
  {
    grasp_motion::Base::rosParamInit();

    std::string ns = nhp_.getNamespace();

    nhp_.param("verbose", verbose_, false);
    nhp_.param("debug_verbose", debug_verbose_, false);

    nhp_.param("pose_fixed_count", pose_fixed_count_, 1.0); //sec
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", pose_fixed_count: " << pose_fixed_count_ <<std::endl;
    nhp_.param("full_contact_count", full_contact_count_, 1.0); //sec
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", full_contact_count: " << full_contact_count_ <<std::endl;

    nhp_.param("grasping_height_threshold", grasping_height_threshold_, 0.01); //m
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", grasping_height_threshold: " << grasping_height_threshold_ <<std::endl;

    nhp_.param("approach_offset_dist", approach_offset_dist_, 0.1); // [m]
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", approach_offset_dist: " << approach_offset_dist_ <<std::endl;
    nhp_.param("approach_delta_angle", approach_delta_angle_, -0.45); // about 25deg
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", approach_delta_angle: " << approach_delta_angle_ <<std::endl;

    nhp_.param("contact_tilt_angle", contact_tilt_angle_, 0.25); //  > 1deg
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", contact_tilt_angle: " << contact_tilt_angle_ <<std::endl;
    nhp_.param("contact_rolling_yaw_angle", contact_rolling_yaw_angle_, 0.2); //  > 1deg
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", contact_rolling_yaw_angle: " << contact_rolling_yaw_angle_ <<std::endl;

    nhp_.param("contact_dist_threshold", contact_dist_threshold_, 0.02); // [m/s]
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", contact_dist_threshold: " << contact_dist_threshold_ <<std::endl;

    nhp_.param("default_yaw_p_gain", default_yaw_p_gain_, 100.0);
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", default_yaw_p_gain: " << default_yaw_p_gain_ <<std::endl;
    nhp_.param("contact_yaw_p_gain", contact_yaw_p_gain_, 1000.0);
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", contact_yaw_p_gain: " << contact_yaw_p_gain_ <<std::endl;

  }

  bool WholeBody::grasp()
  {
    assert(transportator_->phase_ == phase::GRASPING);
    Base::grasp();

    switch(grasp_phase)
      {
      case grasp_motion::IDLE:
        {
          cnt = 0;
          grasp_phase++;
          ROS_WARN("[hydrus grasp motion]: start grasp motion, first transform");
          /* send joint angles */
          sensor_msgs::JointState joint_ctrl_msg;
          joint_ctrl_msg.header.stamp = ros::Time::now();
          for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
            {
              double approach_delta_angle = approach_delta_angle_;
              /* do not extend at center jont */
              int index = std::distance(joints_.begin(), itr);
              if(index == center_joint) approach_delta_angle = 0;
              itr->setTargetAngle(grasp_form_search_method_->getBestTheta().at(index) + approach_delta_angle);
              joint_ctrl_msg.position.push_back(itr->getTargetAngle());
            }
          joint_ctrl_pub_.publish(joint_ctrl_msg);

          return false;
        }
      case grasp_motion::TRANSFORM:
        {
          bool pose_fixed_flag  = true;
          for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
            pose_fixed_flag &= itr->convergence();

          if(pose_fixed_flag && transportator_->positionConvergence() && transportator_->yawConvergence(true))
            {
              if(++cnt > (pose_fixed_count_ * transportator_->func_loop_rate_))
                {
                  ROS_INFO("[hydrus grasp motion]: transform succeeds, shift to descend");
                  grasp_phase ++;
                  cnt = 0; // convergence reset
                  transportator_->uav_target_height_ = transportator_->uav_cog_pos_.z();

                  //aoba
                  //transportator_->phase_ = phase::IDLE;
                }
            }
          return false;
        }
      case grasp_motion::DESCEND:
        {
          transportator_->uav_target_height_ += (transportator_->falling_speed_ / transportator_->func_loop_rate_);
          if(transportator_->uav_target_height_ < transportator_->grasping_height_) transportator_->uav_target_height_ = transportator_->grasping_height_;

          /* only send height nav msg */
          aerial_robot_base::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_pos_z = transportator_->uav_target_height_;
          transportator_->uav_nav_pub_.publish(nav_msg);

          if(fabs(transportator_->grasping_height_ - transportator_->uav_cog_pos_.z()) < grasping_height_threshold_)
            {
              ROS_INFO("[hydrus grasp motion]: descend succeeds, shift to contact");
              grasp_phase++;
              cnt = 0;

              /* send contact yaw control gain */
              sendControlGain(contact_yaw_p_gain_);
            }
          return false;
        }
      case grasp_motion::CONTACT:
        {
          /* to be stauible for the movement of object */
          tf::Vector3 uav_target_cog_acc(0, 0, 0);
          int contact_status;
          double baselink_dist;
          double neighbour_dist;
          bool full_contact = false;
          getContactStatus(contact_status, baselink_dist, neighbour_dist);

          /* rqt_plot debug */
          geometry_msgs::Vector3 contact_state_msg;
          contact_state_msg.x = baselink_dist;
          contact_state_msg.y = neighbour_dist;

          if(object_type == CONVEX_POLYGONAL_COLUMN)
            {
              tf::Vector3 baselink_acc_direct, neighbour_acc_direct;

              tf::Quaternion baselink_contact_rot(0, 0, 0, 1);
              tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_), baselink_contact_rot);
              double baselink_contact_phi = transportator_->uav_cog_yaw_ - (tf::getYaw(baselink_contact_rot) + transportator_->object_yaw_);
              if(baselink_contact_phi > M_PI) baselink_contact_phi -= 2*M_PI;
              if(baselink_contact_phi <-M_PI) baselink_contact_phi += 2*M_PI;

              /* rqt_plot debug */
              contact_state_msg.z = baselink_contact_phi;

              if(baselink_ < neighbour_link)
                {
                  baselink_acc_direct = transportator_->getObject2dTransform().getBasis() * getContactFrame(neighbour_link).getBasis() * tf::Vector3(1, 0, 0);
                  neighbour_acc_direct = transportator_->getObject2dTransform().getBasis() * getContactFrame(baselink_).getBasis() * tf::Vector3(-1, 0, 0);
                }
              else
                {
                  baselink_acc_direct = transportator_->getObject2dTransform().getBasis() * getContactFrame(neighbour_link).getBasis() * tf::Vector3(-1, 0, 0);
                  neighbour_acc_direct = transportator_->getObject2dTransform().getBasis() * getContactFrame(baselink_).getBasis() * tf::Vector3(1, 0, 0);
                }

              if(contact_status == contact_status::BOTH)
                {
                  /* debug: contact distance for baselink  */
                  tf::Vector3 vertex_p;
                  tf::vectorEigenToTF(grasp_form_search_method_->getObjectInfo().at(baselink_)->vertex_p_, vertex_p);
                  double baselink_contact_d = ((getRotorOriginInWordFrame(baselink_) +  transportator_->getObject2dTransform().getBasis() * tf::Matrix3x3(baselink_contact_rot) * tf::Vector3(0, duct_radius, 0)) - transportator_->getObject2dTransform() * vertex_p).length();

                  ROS_WARN("baselink_contact_phi: %f, lower bound: %f, upper bound: %f, baselink_contact_d: %f", baselink_contact_phi, grasp_form_search_method_->getValidLowerBoundPhi().at(baselink_), grasp_form_search_method_->getValidUpperBoundPhi().at(baselink_), baselink_contact_d);

                  if(grasp_form_search_method_->getValidLowerBoundPhi().at(baselink_) < baselink_contact_phi && baselink_contact_phi < grasp_form_search_method_->getValidUpperBoundPhi().at(baselink_))
                    {
                      //ROS_ERROR("convergent");
                      full_contact = true;
                    }
                }
              else
                uav_target_cog_acc = (baselink_acc_direct * (baselink_dist) / (baselink_dist + neighbour_dist) + neighbour_acc_direct * neighbour_dist / (baselink_dist + neighbour_dist) ).normalized() * contact_tilt_angle_ * G;
            }

          if(object_type == CYLINDER)
            {
              tf::Vector3 center_joint_p;
              tf::Vector3 cog2baselink_rotor;
              tf::vectorEigenToTF(uav_kinematics_->getRotorsOriginFromCog().at(baselink_), cog2baselink_rotor);

              if(baselink_ < neighbour_link)
                center_joint_p = transportator_->getUavCog2dTransform() * (cog2baselink_rotor + tf::Vector3(grasp_form_search_method_->getLinkLength()/2, 0, 0));
              else
                center_joint_p = transportator_->getUavCog2dTransform() * (cog2baselink_rotor + tf::Vector3(-grasp_form_search_method_->getLinkLength()/2, 0, 0));

              uav_target_cog_acc = (transportator_->object_2d_pos_ -  center_joint_p).normalized() * contact_tilt_angle_ * G;

              /* rqt_plot debug */
              contact_state_msg.z = transportator_->uav_cog_yaw_;

              switch(contact_status)
                {
                case contact_status::BASELINK:
                  {
                    if(baselink_ < neighbour_link)
                      transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_ + contact_rolling_yaw_angle_;
                    else
                      transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_ - contact_rolling_yaw_angle_;
                    break;
                  }
                case contact_status::NEIGHBOUR:
                  {
                    if(neighbour_link < baselink_)
                      transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_ + contact_rolling_yaw_angle_;
                    else
                      transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_ - contact_rolling_yaw_angle_;
                    break;
                  }
                case contact_status::BOTH:
                  {
                    full_contact = true;
                    break;
                  }
                }
            }

          if(debug_verbose_)
            ROS_INFO("[hydrus grasp motion]: contact motion, target acc: [%f, %f, %f], baselink contact dist: %f, neighbour link contact dist: %f", uav_target_cog_acc.x(), uav_target_cog_acc.y(), uav_target_cog_acc.z(), baselink_dist, neighbour_dist);
          /* rqt_plot debug */
          contact_state_pub_.publish(contact_state_msg);

          if(full_contact)
            {
              if(++cnt > (full_contact_count_ * transportator_->func_loop_rate_))
                {
                  ROS_INFO("[hydrus grasp motion]: full contact succeeds, shift to grasping");
                  grasp_phase++;
                  cnt = 0;

                  /* send roll/pitch i term disable command to d_board */
                  aerial_robot_base::FlightConfigCmd flight_config_cmd;
                  flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::INTEGRATION_CONTROL_OFF_CMD;
                  flight_config_pub_.publish(flight_config_cmd);

                  /* set acc */
                  if(object_type == CONVEX_POLYGONAL_COLUMN)
                    {
                      uav_target_cog_acc = (transportator_->getObject2dTransform().getBasis() * getContactFrame(neighbour_link).getBasis() * tf::Vector3(0, 1, 0) + transportator_->getObject2dTransform().getBasis() * getContactFrame(baselink_).getBasis() * tf::Vector3(0, 1, 0)).normalized() * contact_tilt_angle_ * G;
                    }

                  /* set psi */
                  transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_;
                }
            }

          aerial_robot_base::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.target = aerial_robot_base::FlightNav::COG;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::ACC_MODE;
          nav_msg.target_acc_x = uav_target_cog_acc.x();
          nav_msg.target_acc_y = uav_target_cog_acc.y();
          nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_psi = transportator_->uav_target_cog_yaw_;
          transportator_->uav_nav_pub_.publish(nav_msg);

          return false;
        }
      case grasp_motion::GRASP:
        {
          for(int i = 0; i < joints_.size(); i++)
            if(!joints_.at(i).init_flag_) return false;

          /* nenetti test */
          ROS_INFO( "[hydrus grasp motion]: grasp: joint1: [%f, %f, %d, %f], joint2: [%f, %f, %d, %f], joint3: [%f, %f, %d, %f]", joints_.at(0).current_angle_, joints_.at(0).current_torque_, joints_.at(0).error_, joints_.at(0).target_angle_, joints_.at(1).current_angle_, joints_.at(1).current_torque_, joints_.at(1).error_, joints_.at(1).target_angle_, joints_.at(2).current_angle_, joints_.at(2).current_torque_, joints_.at(2).error_, joints_.at(2).target_angle_);
          int max_level = (baselink_ < uav_kinematics_->getRotorNum() - 1 - baselink_)?uav_kinematics_->getRotorNum() - 1 - baselink_: baselink_;
          /*
          if(joints_.at(2).grasping())
            {
              joints_.at(2).grasping(250);
              ROS_WARN("grasp!!");
            }
          else ROS_INFO("not grasp");
          sensor_msgs::JointState joint_ctrl_msg2;
          joint_ctrl_msg2.header.stamp = ros::Time::now();
          for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
            joint_ctrl_msg2.position.push_back(itr->getTargetAngle());
          joint_ctrl_pub_.publish(joint_ctrl_msg2);
          */
          //return false;
          ///////////////////////

          /* check the rough grasping */
          bool current_level_rough_grasp_flag = true;
          for(int level = 0; level <= grasping_level; level++)
            {
              if (center_joint - level >= 0)
                current_level_rough_grasp_flag &= joints_.at(center_joint - level).grasping();
              if (center_joint + level < uav_kinematics_->getRotorNum() - 1 && level != 0)
                current_level_rough_grasp_flag &= joints_.at(center_joint + level).grasping();
            }

          /* shift to next level */
          if(current_level_rough_grasp_flag && grasping_level < max_level)
            {
              ROS_WARN("grasping: level up");

              /* initizalize */
              grasping_level ++;
              if (center_joint - grasping_level >= 0)
                {
                  joints_.at(center_joint - grasping_level).setTargetAngle(joints_.at(center_joint - grasping_level).current_angle_);
                  joints_.at(center_joint - grasping_level).grasping(0, true);
                }
              if (center_joint + grasping_level < uav_kinematics_->getRotorNum() - 1 && grasping_level != 0)
                {
                  joints_.at(center_joint + grasping_level).setTargetAngle(joints_.at(center_joint + grasping_level).current_angle_);
                  joints_.at(center_joint + grasping_level).grasping(0, true);
                }
            }

          /* check torque based grasp */
          bool force_closure = true;
          if( grasping_level == max_level)
            {
              for(int i = 0; i < joints_.size(); i ++)
                force_closure &= joints_.at(i).grasping(v_grasp_torque(i));
              ROS_ERROR("force_closure: %d, target_torque: %f, %f, %f", force_closure, v_grasp_torque(0), v_grasp_torque(1), v_grasp_torque(2));
            }
          else force_closure = false;

          //aoba
          // force_closure = true;
          // sensor_msgs::JointState joint_ctrl_msg2;
          // joint_ctrl_msg2.header.stamp = ros::Time::now();
          // for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
          //   joint_ctrl_msg2.position.push_back(M_PI/2);
          // joint_ctrl_pub_.publish(joint_ctrl_msg2);

          if(force_closure)
            {
              ROS_ERROR("[hydrus grasp motion]: grasp: force-closure!!!!!!!!!!!");
              /* 1. add extra module: nominal */
              tf::Transform tf_object_origin_to_uav_root;
              tf::Vector3 uav_root_origin;
              tf::vectorEigenToTF(grasp_form_search_method_->getBestJointP().at(0), uav_root_origin);
              tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
              tf::Quaternion uav_root_q(0, 0, 0, 1);
              if(object_type == CONVEX_POLYGONAL_COLUMN)
                tf::quaternionEigenToTF(grasp_form_search_method_->getObjectInfo().at(0)->contact_rot_ * AngleAxisd(grasp_form_search_method_->getBestPhi().at(0), Vector3d::UnitZ()), uav_root_q);
              tf_object_origin_to_uav_root.setRotation(uav_root_q);

              ros::ServiceClient client = nh_.serviceClient<hydrus::AddExtraModule>("/add_extra_module");
              hydrus::AddExtraModule srv;
              srv.request.action = hydrus::AddExtraModule::Request::ADD;
              srv.request.module_name = std::string("object");
              srv.request.parent_link_name = std::string("link1");
              tf::transformTFToMsg(tf_object_origin_to_uav_root.inverse(), srv.request.transform);
              srv.request.inertia = grasp_form_search_method_->getObjectInertia();
              if (client.call(srv))
                {
                  ROS_INFO("suceed to add extra module ");
                }
              else
                {
                  ROS_ERROR("Failed to call service to add extra module");
                }

              /* 2. send rolling yaw control gain */
              sendControlGain(default_yaw_p_gain_);

              /* 3. send roll/pitch i term enable command to d_board */
              aerial_robot_base::FlightConfigCmd flight_config_cmd;
              flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
              flight_config_pub_.publish(flight_config_cmd);

              return true;
            }

          /* send joint angle */
          sensor_msgs::JointState joint_ctrl_msg;
          joint_ctrl_msg.header.stamp = ros::Time::now();
          for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
            joint_ctrl_msg.position.push_back(itr->getTargetAngle());
          joint_ctrl_pub_.publish(joint_ctrl_msg);

          return false;
        }
      default:
        {
          break;
        }
      }
  }

  bool WholeBody::drop()
  {
    if(once_flag)
      {
        once_flag = false;

        /* send joint angles */
        sensor_msgs::JointState joint_ctrl_msg;
        joint_ctrl_msg.header.stamp = ros::Time::now();
        for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
          {
            double approach_delta_angle = approach_delta_angle_;
            /* do not extend at center jont */
            int index = std::distance(joints_.begin(), itr);
            if(index == center_joint) approach_delta_angle = 0;
            itr->setTargetAngle(grasp_form_search_method_->getBestTheta().at(index) + approach_delta_angle);
            joint_ctrl_msg.position.push_back(itr->getTargetAngle());
          }
        joint_ctrl_pub_.publish(joint_ctrl_msg);

        ros::ServiceClient client = nh_.serviceClient<hydrus::AddExtraModule>("/add_extra_module");
        hydrus::AddExtraModule srv;
        srv.request.action = hydrus::AddExtraModule::Request::REMOVE;
        srv.request.module_name = std::string("object");
        srv.request.parent_link_name = std::string("link1");
        if (client.call(srv))
          {
            ROS_INFO("suceed to remove extra module ");
          }
        else
          {
            ROS_ERROR("Failed to call service to add extra module");
          }
      }

    bool pose_fixed_flag  = true;
    for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
      pose_fixed_flag &= itr->convergence();

    if(pose_fixed_flag)
      {
        grasp_phase = grasp_motion::IDLE;
        neighbour_link = 0;
        grasping_level = -1;
      }

    return pose_fixed_flag;
  }

  void WholeBody::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states_msg)
  {
    assert(joint_states_msg->position.size() == uav_kinematics_->getRotorNum() - 1);
    uav_kinematics_->kinematics(*joint_states_msg);

    for(int i = 0; i < joint_states_msg->position.size(); i++)
      {
        joints_.at(i).current_angle_ = joint_states_msg->position[i];

        if(!joints_.at(i).init_flag_)
          {
            joints_.at(i).init_flag_ = true;
            joints_.at(i).target_angle_ = joint_states_msg->position[i];
          }
      }
  }

  void WholeBody::jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg)
  {
    assert(joint_motors_msg->motor_states.size() == uav_kinematics_->getRotorNum() - 1);

    /* debug */
    dynamixel_msgs::MotorStateList joint_motors_debug_msg = *joint_motors_msg;

    for(int i = 0; i < joint_motors_msg->motor_states.size(); i++)
      {
        joints_.at(i).moving_ = joint_motors_msg->motor_states.at(i).moving;
        joints_.at(i).current_torque_ = joint_motors_msg->motor_states.at(i).load;
        joints_.at(i).temperature_ = joint_motors_msg->motor_states.at(i).temperature;
        joints_.at(i).error_ = joint_motors_msg->motor_states.at(i).error;

        /* debug */
        joint_motors_debug_msg.motor_states.at(i).load = (joint_motors_msg->motor_states.at(i).load > 648)?1:(joint_motors_msg->motor_states.at(i).load/648);
      }

    /* debug */
    joint_motors_pub_.publish(joint_motors_debug_msg);
  }

  tf::Vector3 WholeBody::getUavTargetApproach2DPos()
  {
    assert(transportator_->phase_ == phase::APPROACH);

    double r,p,y;
    transportator_->getObject2dTransform().getBasis().getRPY(r, p, y);
    //ROS_INFO("object pos in world frame: [%f, %f, %f], yaw: %f", transportator_->getObject2dTransform().getOrigin().x(), transportator_->getObject2dTransform().getOrigin().y(), transportator_->getObject2dTransform().getOrigin().z(), y);

    /* update the kinematics of hydrus for the approach form */
    sensor_msgs::JointState joint_states;
    int joint_num = uav_kinematics_->getRotorNum() - 1;
    joint_states.position.resize(joint_num, 0);
    joint_states.name.resize(joint_num);
    for(int i = 0; i < joint_num; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        double approach_delta_angle = approach_delta_angle_;
        if(i == center_joint) approach_delta_angle = 0;
        joint_states.position.at(i) = grasp_form_search_method_->getBestTheta().at(i) + approach_delta_angle;
        joint_states.name.at(i) = std::string("joint") + joint_no.str();
      }
    uav_kinematics_->kinematics(joint_states);

    int first_contact_index = baselink_;
    /* special for cylinder */
    if(object_type == CYLINDER) first_contact_index = 0;

    tf::Transform baselink_rotor_frame_in_object_frame;
    baselink_rotor_frame_in_object_frame = getContactFrame(first_contact_index) * tf::Transform(tf::createQuaternionFromYaw(grasp_form_search_method_->getBestPhi().at(baselink_)), tf::Vector3(0, -duct_radius, 0));

    tf::Vector3 approach_offset_in_object_frame;
    if(object_type == CONVEX_POLYGONAL_COLUMN)
      {
        if(neighbour_link < baselink_)
          tf::vectorEigenToTF(grasp_form_search_method_->getBestContactRot().at(neighbour_link) * Vector3d(1, 0, 0) + grasp_form_search_method_->getBestContactRot().at(baselink_) * Vector3d(-1, 0, 0), approach_offset_in_object_frame);
        else
          tf::vectorEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_) * Vector3d(1, 0, 0) + grasp_form_search_method_->getBestContactRot().at(neighbour_link) * Vector3d(-1, 0, 0), approach_offset_in_object_frame);

        approach_offset_in_object_frame = approach_offset_in_object_frame.normalize() * approach_offset_dist_;
      }
    else if(object_type == CYLINDER)
      {
        double tilt_angle = grasp_form_search_method_->getBestTheta().at(center_joint) / 2;
        if(baselink_ < neighbour_link)
          approach_offset_in_object_frame = tf::Matrix3x3(tf::createQuaternionFromYaw(tilt_angle - M_PI/2)) * tf::Vector3(approach_offset_dist_, 0, 0);
        else
          approach_offset_in_object_frame = tf::Matrix3x3(tf::createQuaternionFromYaw(-tilt_angle - M_PI/2)) * tf::Vector3(approach_offset_dist_, 0, 0);
      }
    ROS_INFO("approach_offset_in_object_frame: [%f, %f, %f]", approach_offset_in_object_frame.x(), approach_offset_in_object_frame.y(), approach_offset_in_object_frame.z());
 
    tf::Vector3 cog2baselink_rotor;
    tf::vectorEigenToTF(uav_kinematics_->getRotorsOriginFromCog().at(baselink_), cog2baselink_rotor);
    //ROS_INFO("cog2baselink_rotr: [%f, %f, %f]", cog2baselink_rotor.x(), cog2baselink_rotor.y(), cog2baselink_rotor.z());

    return transportator_->getObject2dTransform() * (baselink_rotor_frame_in_object_frame * (-cog2baselink_rotor) + approach_offset_in_object_frame);
  }

  double WholeBody::getUavTargetApproachYaw()
  {
    tf::Quaternion first_contact_rot;
    tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_), first_contact_rot);
    return transportator_->object_yaw_ + tf::getYaw(first_contact_rot) +grasp_form_search_method_->getBestPhi().at(baselink_);
  }

  void WholeBody::sendControlGain(double yaw_p_gain)
  {

    dynamic_reconfigure::Reconfigure srv;

    dynamic_reconfigure::Config conf;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::DoubleParameter double_param;

    bool_param.name = "lqi_gain_flag";
    bool_param.value = true;
    conf.bools.push_back(bool_param);

    double_param.name = "q_yaw";
    double_param.value = yaw_p_gain;
    conf.doubles.push_back(double_param);

    srv.request.config = conf;

    if (control_gain_client_.call(srv))
      ROS_INFO("succeed to modify the control gain ");
    else
      ROS_ERROR("Failed to call service to control gain");
  }

  void WholeBody::getContactStatus(int& contact_status, double& baselink_dist, double& neighbour_link_dist)
  {
    switch(object_type)
      {
      case CONVEX_POLYGONAL_COLUMN:
        {
          baselink_dist = -((transportator_->getObject2dTransform() * getContactFrame(baselink_)).inverse() * getRotorOriginInWordFrame(baselink_)).y() - duct_radius;
          neighbour_link_dist = -((transportator_->getObject2dTransform() * getContactFrame(neighbour_link)).inverse() * getRotorOriginInWordFrame(neighbour_link)).y() - duct_radius;
          break;
        }
      case CYLINDER:
        {
          baselink_dist = (getRotorOriginInWordFrame(baselink_) - transportator_->object_2d_pos_).length() - duct_radius - object_radius;
          neighbour_link_dist = (getRotorOriginInWordFrame(neighbour_link) - transportator_->object_2d_pos_).length() - duct_radius - object_radius;
          break;
        }
      default:
        {
          break;
        }
      }

    if(baselink_dist < contact_dist_threshold_ && neighbour_link_dist < contact_dist_threshold_)
      contact_status = contact_status::BOTH;

    if(baselink_dist < contact_dist_threshold_ && neighbour_link_dist > contact_dist_threshold_)
      contact_status = contact_status::BASELINK;

    if(baselink_dist > contact_dist_threshold_ && neighbour_link_dist < contact_dist_threshold_)
      contact_status = contact_status::NEIGHBOUR;

    if(baselink_dist > contact_dist_threshold_ && neighbour_link_dist > contact_dist_threshold_)
      contact_status = contact_status::NO;
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grasp_motion::WholeBody, grasp_motion::Base);
