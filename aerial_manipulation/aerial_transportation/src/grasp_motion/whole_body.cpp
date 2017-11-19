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

    int phase = grasp_motion::IDLE;

    /* for convex polygon rolling */
    int neighbour_link = 0;
    int center_joint = 0;
    int roll_phase = roll_motion::IDLE;
    int prev_roll_phase = roll_motion::IDLE;
    double neighbour_link_target_yaw = 0;

    double duct_radius = 0;
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
      joints_.push_back(grasp_motion::JointHandle(nh_, nhp_, grasp_form_search_method_->getBestTheta().at(i), i));

    /* init for the polygon roll motion */
    baselink_ = grasp_form_search_method_->getBaselink();
    duct_radius = grasp_form_search_method_->getDuctRadius();
    object_type = grasp_form_search_method_->getObjectType();
    if(uav_kinematics_->getRotorNum() > baselink_)
      {
        neighbour_link = baselink_ + 1;
        center_joint = baselink_;
      }
    else
      {
        neighbour_link = baselink_ - 1;
        center_joint = neighbour_link;
      }
    ROS_INFO("[hydrus grasp motion] polygon rolling motion, neighbour link: %d, center joint: :%d", neighbour_link, center_joint);

    /* torque grasp */
    double torque_grasp_min_torque_rate;
    nhp_.param("torque_grasp_min_torque_rate", torque_grasp_min_torque_rate, 1.1); // [m]
    v_grasp_torque = grasp_form_search_method_->getBestTau();
    v_grasp_torque *= (joints_.at(0).grasp_min_torque_ * torque_grasp_min_torque_rate / v_grasp_torque.minCoeff());
    if(v_grasp_torque.maxCoeff() > joints_.at(0).grasp_max_torque_) ROS_FATAL("the grasp max torque %f exceed the valid value %f, the min is %f ", v_grasp_torque.maxCoeff(), joints_.at(0).grasp_max_torque_, v_grasp_torque.minCoeff());

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

    //strong yaw initially. TODO yaw(LQI control)
  }

  void WholeBody::rosParamInit()
  {
    grasp_motion::Base::rosParamInit();

    std::string ns = nhp_.getNamespace();

    nhp_.param("verbose", verbose_, false);
    nhp_.param("debug_verbose", debug_verbose_, false);

    nhp_.param("pose_fixed_count", pose_fixed_count_, 1.0); //sec
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", pose_fixed_count: " << pose_fixed_count_ <<std::endl;
    nhp_.param("grasping_height_threshold", grasping_height_threshold_, 0.01); //m
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", grasping_height_threshold: " << grasping_height_threshold_ <<std::endl;

    nhp_.param("approach_offset_dist", approach_offset_dist_, 0.1); // [m]
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", approach_offset_dist: " << approach_offset_dist_ <<std::endl;

    nhp_.param("approach_acc_tilt_angle", approach_acc_tilt_angle_, 0.34); // about 2deg
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", approach_acc_tilt_angle: " << approach_acc_tilt_angle_ <<std::endl;
    nhp_.param("contact_tilt_angle", contact_tilt_angle_, 0.25); //  > 1deg
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", contact_tilt_angle: " << contact_tilt_angle_ <<std::endl;
    nhp_.param("contact_vel_threshold", contact_vel_threshold_, 0.02); // [m/s]
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", contact_vel_threshold: " << contact_vel_threshold_ <<std::endl;
    nhp_.param("rolling_angle_threshold", rolling_angle_threshold_, 0.3); // [m/s]
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", rolling_angle_threshold: " << rolling_angle_threshold_ <<std::endl;
    nhp_.param("rolling_yaw_threshold", rolling_yaw_threshold_, 0.1); // [rad] IMPORTANT!!!
    if(verbose_) std::cout << "[hydrus grasp motion] ns: " << ns << ", rolling_yaw_threshold: " << rolling_yaw_threshold_ <<std::endl;
  }

  bool WholeBody::grasp()
  {
    assert(transportator_->phase_ == phase::GRASPING);
    Base::grasp();

    switch(phase)
      {
      case grasp_motion::IDLE:
        {
          cnt = 0;
          phase++;
          ROS_WARN("[hydrus grasp motion]: start grasp motion, first transform");
          /* send joint angles */
          sensor_msgs::JointState joint_ctrl_msg;
          joint_ctrl_msg.header.stamp = ros::Time::now();
          for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
            {
              itr->approaching();
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

          if(pose_fixed_flag && transportator_->positionConvergence() && transportator_->yawConvergence())
            {
              if(++cnt > (pose_fixed_count_ * transportator_->func_loop_rate_))
                {
                  ROS_INFO("[hydrus grasp motion]: transform succeeds, shift to descend");
                  phase ++;
                  cnt = 0; // convergence reset
                  transportator_->uav_target_height_ = transportator_->uav_cog_pos_.z();
                }
            }
          return false;
        }
      case grasp_motion::DESCEND:
        {
          transportator_->uav_target_height_ += (transportator_->falling_speed_ / transportator_->func_loop_rate_);
          if(transportator_->uav_target_height_ < transportator_->object_height_) transportator_->uav_target_height_ = transportator_->object_height_;

          /* only send height nav msg */
          aerial_robot_base::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.pos_z_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
          nav_msg.target_pos_z = transportator_->uav_target_height_;
          transportator_->uav_nav_pub_.publish(nav_msg);

          if(fabs(transportator_->object_height_ - transportator_->uav_cog_pos_.z()) < grasping_height_threshold_)
            {
              ROS_INFO("[hydrus grasp motion]: descend succeeds, shift to contact");
              phase++;
              cnt = 0;
              transportator_->uav_target_cog_2d_pos_ = getUavTargetApproach2DPos(transportator_->object_2d_pos_, transportator_->object_yaw_);
            }
          return false;
        }
      case grasp_motion::CONTACT:
        {
          /* to be stauible for the movement of object */
          tf::Vector3 uav_target_cog_acc;
          if(grasp_form_search_method_->getObjectType() == CONVEX_POLYGONAL_COLUMN)
            {
#if 1 /* guidance */
              uav_target_cog_acc =  (transportator_->uav_target_cog_2d_pos_ - transportator_->uav_cog_2d_pos_).normalize() * approach_acc_tilt_angle_ * G;
#else
              tf::Quaternion first_contact_rot;
              tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_), first_contact_rot);
              uav_target_cog_acc = tf::Matrix3x3(tf::createQuaternionFromYaw(transportator_->object_yaw_) * first_contact_rot * tf::createQuaternionFromYaw(M_PI/2)) * tf::Vector3(approach_acc_tilt_angle_ * G, 0, 0);
#endif
            }
          if(grasp_form_search_method_->getObjectType() == CYLINDER)
            {

              uav_target_cog_acc = (transportator_->object_2d_pos_ - getBaseLinkRotorOrigin()).normalize() * approach_acc_tilt_angle_ * G;
            }

          aerial_robot_base::FlightNav nav_msg;
          nav_msg.header.stamp = ros::Time::now();
          nav_msg.target = aerial_robot_base::FlightNav::COG;
          nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::ACC_MODE;
          nav_msg.target_pos_x = uav_target_cog_acc.x();
          nav_msg.target_pos_y = uav_target_cog_acc.y();
          transportator_->uav_nav_pub_.publish(nav_msg);

          if(debug_verbose_)
            ROS_INFO("[hydrus grasp motion]: first contact motion, target acc: [%f, %f, %f], uav_vel: [%f, %f, %f], vel_norm: %f", uav_target_cog_acc.x(), uav_target_cog_acc.y(), uav_target_cog_acc.z(),(transportator_->uav_cog_vel_).x(), (transportator_->uav_cog_vel_).y(), (transportator_->uav_cog_vel_).z(), (transportator_->uav_cog_vel_).length() );

          if((transportator_->uav_cog_vel_).length() <  contact_vel_threshold_)
            {
              if(++cnt > (pose_fixed_count_ * transportator_->func_loop_rate_))
                {
                  ROS_INFO("[hydrus grasp motion]: first contact succeeds, shift to rolling");
                  phase++;
                  cnt = 0;

                  /* send roll/pitch i term disable command to d_board */
                  aerial_robot_base::FlightConfigCmd flight_config_cmd;
                  flight_config_cmd.cmd = aerial_robot_base::FlightConfigCmd::INTEGRATION_CONTROL_OFF_CMD;
                  flight_config_pub_.publish(flight_config_cmd);
                }
            }
          return false;
        }
      case grasp_motion::ROLL:
        {
          rollMotion();
          return false;
        }
      case grasp_motion::GRASP:
        {
          int max_level = (baselink_ < uav_kinematics_->getRotorNum() - 1 - baselink_)?uav_kinematics_->getRotorNum() - 1 - baselink_: baselink_;

          /* check the rough grasping */
          bool current_level_rough_grasp_flag = true;
          if(grasping_level >= 0)
            {
              int index = baselink_- grasping_level;
              if (index > 0) current_level_rough_grasp_flag &= joints_.at(index - 1).grasping();
              index = baselink_ + grasping_level;
              if (index < uav_kinematics_->getRotorNum() - 1) current_level_rough_grasp_flag &= joints_.at(index).grasping();
            }

          /* shift to next level */
          if(current_level_rough_grasp_flag)
            {
              /* initizalize */
              grasping_level ++;
              int index = baselink_- grasping_level;
              if (index > 0) joints_.at(index - 1).grasping(0, true);
              index = baselink_ + grasping_level;
              if (index < uav_kinematics_->getRotorNum() - 1) joints_.at(index).grasping(0, true);
            }

          /* check torque based grasp */
          bool force_closure = true;
          if( grasping_level == max_level)
            {
              for(int i = 0; i < joints_.size(); i ++)
                force_closure &= joints_.at(i).grasping(v_grasp_torque(i));
            }
          else force_closure = false;

          if(force_closure)
            {
              /* nominal */
              tf::Transform tf_object_origin_to_uav_root;
              tf::Vector3 uav_root_origin;
              tf::vectorEigenToTF(grasp_form_search_method_->getBestJointP().at(0), uav_root_origin);
              tf_object_origin_to_uav_root.setOrigin(uav_root_origin);
              tf::Quaternion uav_root_q;
              tf::quaternionEigenToTF(grasp_form_search_method_->getObjectInfo().at(0)->contact_rot_ * AngleAxisd(grasp_form_search_method_->getBestPhi().at(0), Vector3d::UnitZ()), uav_root_q);
              tf_object_origin_to_uav_root.setRotation(uav_root_q);
              //1. TODO object inertia in python

              ros::ServiceClient client = nh_.serviceClient<hydrus::AddExtraModule>("add_extra_module");
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
            itr->approaching();
            joint_ctrl_msg.position.push_back(itr->getTargetAngle());
          }
        joint_ctrl_pub_.publish(joint_ctrl_msg);

      }

    bool pose_fixed_flag  = true;
    for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
      pose_fixed_flag &= itr->convergence();

    if(pose_fixed_flag)
      {
        phase = grasp_motion::IDLE;
        neighbour_link = 0;
        roll_phase = roll_motion::IDLE;
        int prev_roll_phase = roll_motion::IDLE;
        neighbour_link_target_yaw = 0;
        grasping_level = -1;
      }

    return pose_fixed_flag;
  }

  void WholeBody::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_states_msg)
  {
    assert(joint_states_msg->position.size() == uav_kinematics_->getRotorNum() - 1);

    uav_kinematics_->kinematics(*joint_states_msg);

    for(int i = 0; i < joint_states_msg->position.size(); i++)
      joints_.at(i).current_angle_ = joint_states_msg->position[i];
  }

  void WholeBody::jointMotorStatusCallback(const dynamixel_msgs::MotorStateListConstPtr& joint_motors_msg)
  {
    assert(joint_motors_msg->motor_states.size() == uav_kinematics_->getRotorNum() - 1);

    for(int i = 0; i < joint_motors_msg->motor_states.size(); i++)
      {
        joints_.at(i).moving_ = joint_motors_msg->motor_states.at(i).moving;
        joints_.at(i).current_torque_ = joint_motors_msg->motor_states.at(i).load;
        joints_.at(i).temperature_ = joint_motors_msg->motor_states.at(i).temperature;
        joints_.at(i).error_ = joint_motors_msg->motor_states.at(i).error;
      }
  }

  tf::Vector3 WholeBody::getUavTargetApproach2DPos(tf::Vector3 object_2d_pos, double object_yaw)
  {
    /* update the kinematics of hydrus for the approach form */
    sensor_msgs::JointState joint_states;
    int joint_num = uav_kinematics_->getRotorNum() - 1;
    joint_states.position.resize(joint_num, 0);
    joint_states.name.resize(joint_num);
    for(int i = 0; i < joint_num; i++)
      {
        std::stringstream joint_no;
        joint_no << i + 1;
        joint_states.position.at(i) = joints_.at(i).approach_angle_;
        joint_states.name.at(i) = std::string("joint") + joint_no.str();
      }

    uav_kinematics_->kinematics(joint_states);

    tf::Vector3 first_contact_p;
    tf::vectorEigenToTF(grasp_form_search_method_->getBestContactP().at(baselink_), first_contact_p);
    tf::Quaternion first_contact_rot;
    tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_), first_contact_rot);

    tf::Transform object_frame(tf::createQuaternionFromYaw(object_yaw), object_2d_pos);
    tf::Transform first_contact_frame(first_contact_rot, first_contact_p);

    tf::Transform baselink_rotor_frame;
    if(transportator_->phase_ == phase::APPROACH)
      baselink_rotor_frame = tf::Transform(tf::createQuaternionFromYaw(grasp_form_search_method_->getBestPhi().at(baselink_)), tf::Vector3(0, -approach_offset_dist_ - duct_radius, 0));
    else if(transportator_->phase_ == phase::GRASPING)
      {
        /* special process for first contact, half of the radius */
        baselink_rotor_frame = tf::Transform(tf::createQuaternionFromYaw(grasp_form_search_method_->getBestPhi().at(baselink_)), tf::Vector3(0, -duct_radius/2, 0));
      }
    else
      {
        ROS_FATAL("[hydrus grasp motion] getUavTargetApproach2DPos, no respond target pos for phase:%d, grasp phase: %d", transportator_->phase_, phase);
        return object_2d_pos;
      }

    tf::Vector3 cog2baselink_rotor;
    tf::vectorEigenToTF(uav_kinematics_->getRotorsOriginFromCog().at(baselink_), cog2baselink_rotor);

    return object_frame * first_contact_frame * baselink_rotor_frame * (-cog2baselink_rotor);
  }

  double WholeBody::getUavTargetApproachYaw(double object_yaw)
  {
    tf::Quaternion first_contact_rot;
    tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_), first_contact_rot);
    return object_yaw + tf::getYaw(first_contact_rot) +grasp_form_search_method_->getBestPhi().at(baselink_);
  }

  bool WholeBody::rollMotion()
  {
    double baselink_contact_phi = 0;
    double baselink_contact_phi_err = 0;
    double baselink_valid_lower_phi = grasp_form_search_method_->getValidLowerBoundPhi().at(baselink_);
    double baselink_valid_upper_phi = grasp_form_search_method_->getValidUpperBoundPhi().at(baselink_);
    assert(baselink_valid_lower_phi < baselink_valid_upper_phi);

    double baselink_contact_d = 0;
    double baselink_contact_d_err = 0;
    double baselink_valid_upper_contact_d = 0;
    double baselink_valid_lower_contact_d = 0;

    tf::Vector3 baselink_contact_p(0, 0, 0);
    tf::Quaternion baselink_contact_rot(0, 0, 0, 1);
    tf::Vector3 baselink_vertex_p(0, 0, 0);
    double baselink_side_len = 0;

    tf::Vector3 uav_target_cog_acc(0, 0, 0);

    grasp_form_search::VertexHandlePtr baselink_vertex_hp = grasp_form_search_method_->getObjectInfo().at((grasp_form_search_method_->getBestStartSide() + baselink_) % grasp_form_search_method_->getSideNum());
    grasp_form_search::VertexHandlePtr neighbour_vertex_hp = grasp_form_search_method_->getObjectInfo().at((grasp_form_search_method_->getBestStartSide() + neighbour_link) % grasp_form_search_method_->getSideNum());

    if(object_type == CONVEX_POLYGONAL_COLUMN)
      {
        tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(baselink_), baselink_contact_rot);
        baselink_contact_phi = transportator_->uav_cog_yaw_ - (tf::getYaw(baselink_contact_rot) + transportator_->object_yaw_);
        baselink_contact_phi_err = grasp_form_search_method_->getBestPhi().at(baselink_) - baselink_contact_phi;

        baselink_contact_p = getBaseLinkRotorOrigin() + tf::Matrix3x3(tf::createQuaternionFromYaw(transportator_->uav_cog_yaw_ - baselink_contact_phi)) * tf::Vector3(0, grasp_form_search_method_->getDuctRadius(), 0);

        tf::Vector3 vertex_local_p;
        tf::vectorEigenToTF(baselink_vertex_hp->vertex_p_, vertex_local_p);
        baselink_vertex_p = transportator_->getObjectTransform() * vertex_local_p;
        baselink_contact_d = (baselink_contact_p - baselink_vertex_p).length();
        baselink_contact_d_err = grasp_form_search_method_->getBestContactD().at(baselink_) - baselink_contact_d;

        baselink_valid_lower_contact_d = grasp_form_search_method_->getValidLowerBoundContactD().at(baselink_);
        baselink_valid_upper_contact_d = grasp_form_search_method_->getValidUpperBoundContactD().at(baselink_);
        baselink_side_len = baselink_vertex_hp->len_;

        assert(baselink_valid_lower_contact_d < baselink_valid_upper_contact_d);

        uav_target_cog_acc = tf::Matrix3x3(tf::createQuaternionFromYaw(transportator_->object_yaw_) * baselink_contact_rot * tf::createQuaternionFromYaw(M_PI/2)) * tf::Vector3(approach_acc_tilt_angle_ * G, 0, 0);

        if(debug_verbose_)
          ROS_INFO("[hydrus grasp motion] polygon rolling motion, baselink_contact_p: [%f, %f, %f], vertex_p: [%f, %f, %f], contact_phi: %f, contact_d: %f, best_contact_phi: %f, best_contact_d: %f, uav_target_cog_acc: [%f, %f]", baselink_contact_p.x(), baselink_contact_p.y(), baselink_contact_p.z(), baselink_vertex_p.x(), baselink_vertex_p.y(), baselink_vertex_p.z(), baselink_contact_phi, baselink_contact_d, grasp_form_search_method_->getBestPhi().at(baselink_), grasp_form_search_method_->getBestContactD().at(baselink_), uav_target_cog_acc.x(), uav_target_cog_acc.y());

      }
    if(object_type  == CYLINDER)
      {
        uav_target_cog_acc = (transportator_->object_2d_pos_ - getBaseLinkRotorOrigin()).normalize() * approach_acc_tilt_angle_ * G;
      }

    auto contactPointValid = [&]() -> bool {
      if(baselink_contact_phi >= baselink_valid_lower_phi && baselink_contact_phi <= baselink_valid_upper_phi)
        {
          if(object_type == CONVEX_POLYGONAL_COLUMN)
            {
              if(baselink_contact_d >= baselink_valid_lower_contact_d && baselink_contact_d <= baselink_valid_upper_contact_d)
                {
                  ROS_WARN("[hydrus grasp motion] polygon rolling motion, contact point is valid, shift to grasp motion");
                  phase++;
                  assert(phase == grasp_motion::GRASP);
                  return true;
                }
              else return false;
            }
          if(object_type  == CYLINDER)
            {
              ROS_WARN("[hydrus grasp motion] cylinder rolling motion, contact point is valid, shift to grasp motion");
              phase++;
              assert(phase == grasp_motion::GRASP);
              for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
                itr->grasping(0, true);
              return true;
            }
        }
      else return false;
    };

    auto rollingAngleCalc = [&]() -> double {

      if(object_type == CONVEX_POLYGONAL_COLUMN)
        {
          /* obtain the quadratic function */
          /* w_1 * (contact_d_err + duct_radius * delta_phi)^2 + w_2 * (contact_phi_err + delta_phi)^2 */
          /* we set w_1 = 1/ r^2 to set the same level of the d and phi */
          /*
            double a = 2;
            double b = 2 * (contact_d_err / duct_radius_ + contact_phi_err);
            double c = (contact_d_err / duct_radius_) * (contact_d_err / duct_radius_) + contact_phi_err * contact_phi_err;
          */
          double target_delta_phi = - (baselink_contact_d_err / duct_radius + baselink_contact_phi_err) / 2;
          double delta_phi_lower_bound = (-baselink_contact_d / duct_radius > -rolling_angle_threshold_)?-baselink_contact_d / duct_radius: -rolling_angle_threshold_;
          double delta_phi_upper_bound = (baselink_side_len - baselink_contact_d / duct_radius > rolling_angle_threshold_)?rolling_angle_threshold_: baselink_side_len- baselink_contact_d / duct_radius;

          if(target_delta_phi < delta_phi_lower_bound)
            target_delta_phi = delta_phi_lower_bound;
          if(target_delta_phi > delta_phi_lower_bound)
            target_delta_phi = delta_phi_upper_bound;

          return target_delta_phi;
        }
      if(object_type == CYLINDER)
        {
          tf::Vector3 head_vector = (transportator_->object_2d_pos_ - getBaseLinkRotorOrigin());
          return  transportator_->uav_cog_yaw_ - (atan2(head_vector.y(), head_vector.x()) - M_PI/2);
        }
    };

    switch(roll_phase)
      {
      case roll_motion::IDLE:
        {
          if(!contactPointValid())
            {
              transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_ + rollingAngleCalc();
              roll_phase = roll_motion::BASELINK_ROLL;
            }
          break;
        }
      case roll_motion::BASELINK_ROLL:
        {
          if(contactPointValid()) break;

          if(object_type == CONVEX_POLYGONAL_COLUMN)
            {
              if(fabs(transportator_->uav_target_cog_yaw_ - transportator_->uav_cog_yaw_) < rolling_yaw_threshold_)
                {
                  joints_.at(center_joint).grasping(0, true);
                  prev_roll_phase = roll_motion::BASELINK_ROLL;
                  roll_phase = roll_motion::FULL_CONTACT;
                  ROS_INFO("[hydrus grasp motion] polygon rolling motion, baselink rolling convergent, shift to full contact for neighbour link rolling");
                }
            }
          break;
        }
      case roll_motion::FULL_CONTACT:
        {
          assert(object_type == CONVEX_POLYGONAL_COLUMN);

          if(joints_.at(center_joint).grasping())
            {
              ROS_INFO("[hydrus grasp motion] polygon rolling motion, complete full contact");

              if(prev_roll_phase = roll_motion::BASELINK_ROLL)
                {
                  /* calculate the neighbour rolling angle */
                  double center_joint_angle = (neighbour_link - baselink_) * joints_.at(center_joint).current_angle_;
                  double vertex_psi = (baselink_ > neighbour_link)?baselink_vertex_hp->psi_:neighbour_vertex_hp->psi_;
                  double neighbour_contact_phi = center_joint_angle + baselink_contact_phi - vertex_psi;
                  tf::Vector3 vertex_local_p;
                  tf::vectorEigenToTF(neighbour_vertex_hp->vertex_p_, vertex_local_p);
                  tf::Vector3 neighbour_vertex_p = transportator_->getObjectTransform() * vertex_local_p;
                  tf::Vector3 cog2neighbour_link_rotor;
                  tf::vectorEigenToTF(uav_kinematics_->getRotorsOriginFromCog().at(neighbour_link), cog2neighbour_link_rotor);
                  tf::Vector3 neighbour_contact_p = transportator_->getUavCogTransform() * cog2neighbour_link_rotor + tf::Matrix3x3(tf::createQuaternionFromYaw(transportator_->uav_cog_yaw_ + center_joint_angle - neighbour_contact_phi)) * tf::Vector3(0, grasp_form_search_method_->getDuctRadius(), 0);

                  double neighbour_contact_d = (neighbour_contact_p - neighbour_vertex_p).length();
                  double neighbour_side_len = neighbour_vertex_hp->len_;

                  double neighbour_delta_phi_lower_bound = (-neighbour_contact_d / duct_radius > -rolling_angle_threshold_)?-neighbour_contact_d / duct_radius: -rolling_angle_threshold_;

                  double neighbour_delta_phi_upper_bound = (neighbour_side_len - neighbour_contact_d / duct_radius > rolling_angle_threshold_)?rolling_angle_threshold_: neighbour_side_len - neighbour_contact_d / duct_radius;
                  assert(neighbour_delta_phi_lower_bound < neighbour_delta_phi_upper_bound);

                  double best_neighbour_link_phi = 0;
                  double min_cost = 1e6;
                  for(double delta_phi = neighbour_delta_phi_lower_bound;  delta_phi < neighbour_delta_phi_upper_bound; delta_phi+= 0.01) //hard coding: about 0.5[deg]
                    {
                      double theta;
                      Vector3d joint_p;
                      if(neighbour_link < baselink_)
                        {
                          if(grasp_form_search_method_->convexPolygonNeighbourContacts(neighbour_contact_d + duct_radius * delta_phi, neighbour_contact_phi + delta_phi, vertex_psi, neighbour_side_len, baselink_side_len, theta , baselink_contact_phi, joint_p, baselink_contact_d)) continue;
                        }
                      else
                        {
                          if(grasp_form_search_method_->convexPolygonNeighbourContacts(neighbour_side_len - (neighbour_contact_d + duct_radius * delta_phi), - neighbour_contact_phi - delta_phi, vertex_psi, neighbour_side_len, baselink_side_len, theta , baselink_contact_phi, joint_p, baselink_contact_d)) continue;
                          baselink_contact_phi *= -1;
                          baselink_contact_d = baselink_side_len - baselink_contact_d;
                        }
                      baselink_contact_phi_err = grasp_form_search_method_->getBestPhi().at(baselink_) - baselink_contact_phi;
                      baselink_contact_d_err = grasp_form_search_method_->getBestContactD().at(baselink_) - baselink_contact_d;
                      double target_delta_phi = rollingAngleCalc();
                      double cost = std::pow(target_delta_phi + baselink_contact_d_err / duct_radius, 2) + std::pow(target_delta_phi +  baselink_contact_phi_err, 2);

                      if(cost < min_cost)
                        {
                          best_neighbour_link_phi = neighbour_contact_phi + delta_phi;
                          min_cost = cost;
                        }
                    }

                  tf::Quaternion neighbour_contact_rot;
                  tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(neighbour_link), neighbour_contact_rot);
                  neighbour_link_target_yaw = transportator_->object_yaw_ + tf::getYaw(neighbour_contact_rot) + best_neighbour_link_phi;

                  roll_phase = roll_motion::NEIGHBOUR_ROLL;
                  ROS_INFO("[hydrus grasp motion] polygon rolling motion, shift to neighbour rolling phase, neighbour_link_target_yaw: %f, best_neighbour_link_phi: %f, min_cost: %f", neighbour_link_target_yaw, best_neighbour_link_phi, min_cost);
                }
              else if(prev_roll_phase = roll_motion::NEIGHBOUR_ROLL)
                {
                  /* calculate the baselink rolling angle */
                  transportator_->uav_target_cog_yaw_ = transportator_->uav_cog_yaw_ + rollingAngleCalc();
                  roll_phase = roll_motion::BASELINK_ROLL;
                  ROS_INFO("[hydrus grasp motion] polygon rolling motion, shift to baselink rolling phase");
                }

              for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
                itr->approaching();
            }
          break;
        }
      case roll_motion::NEIGHBOUR_ROLL:
        {
          assert(object_type == CONVEX_POLYGONAL_COLUMN);

          /* convert to target baselink yaw*/
          transportator_->uav_target_cog_yaw_ = neighbour_link_target_yaw - (neighbour_link - baselink_) * joints_.at(center_joint).current_angle_;

          /* CHECK neighbour not baselink(cog) */
          double neighbour_link_yaw = transportator_->uav_cog_yaw_ + (neighbour_link - baselink_) * joints_.at(center_joint).current_angle_;

          if(fabs(neighbour_link_target_yaw - neighbour_link_yaw) < rolling_yaw_threshold_)
            {
              joints_.at(center_joint).grasping(0, true);
              prev_roll_phase = roll_motion::NEIGHBOUR_ROLL;
              roll_phase = roll_motion::FULL_CONTACT;
              ROS_INFO("[hydrus grasp motion] polygon rolling motion, neighbour link rolling convergent, shift to full contact for baselink rolling");
            }

          tf::Quaternion nieghbour_link_contact_rot;
          tf::quaternionEigenToTF(grasp_form_search_method_->getBestContactRot().at(neighbour_link), nieghbour_link_contact_rot);
          uav_target_cog_acc = tf::Matrix3x3(tf::createQuaternionFromYaw(transportator_->object_yaw_) * nieghbour_link_contact_rot * tf::createQuaternionFromYaw(M_PI/2)) * tf::Vector3(approach_acc_tilt_angle_ * G, 0, 0);
          break;
        }
      }

    /* send flight nav */
    aerial_robot_base::FlightNav nav_msg;
    nav_msg.header.stamp = ros::Time::now();
    nav_msg.target = aerial_robot_base::FlightNav::COG;
    nav_msg.pos_xy_nav_mode = aerial_robot_base::FlightNav::ACC_MODE;
    nav_msg.target_pos_x = uav_target_cog_acc.x();
    nav_msg.target_pos_y = uav_target_cog_acc.y();
    nav_msg.psi_nav_mode = aerial_robot_base::FlightNav::POS_MODE;
    nav_msg.target_psi = transportator_->uav_target_cog_yaw_;
    transportator_->uav_nav_pub_.publish(nav_msg);

    /* send joint angle */
    sensor_msgs::JointState joint_ctrl_msg;
    joint_ctrl_msg.header.stamp = ros::Time::now();
    for(auto itr = joints_.begin(); itr != joints_.end(); ++itr)
      joint_ctrl_msg.position.push_back(itr->getTargetAngle());
    joint_ctrl_pub_.publish(joint_ctrl_msg);

  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(grasp_motion::WholeBody, grasp_motion::Base);
