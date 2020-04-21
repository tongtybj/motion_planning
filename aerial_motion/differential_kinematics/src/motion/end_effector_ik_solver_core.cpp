// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <differential_kinematics/motion/end_effector_ik_solver_core.h>

using namespace differential_kinematics;

using CostContainer = std::vector<boost::shared_ptr<cost::Base> >;
using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base> >;


EndEffectorIKSolverCore::EndEffectorIKSolverCore(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<HydrusRobotModel> robot_model_ptr, bool simulation): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr)
  {
    planner_core_ptr_ = boost::shared_ptr<Planner> (new Planner(nh, nhp, robot_model_ptr));

    baselink_name_ = robot_model_ptr_->getBaselinkName();

    /* simulation: check the validation of end-effector ik sovler without dynamics */
    if(simulation)
      {
        planner_core_ptr_->registerMotionFunc(std::bind(&EndEffectorIKSolverCore::motionFunc, this));

        std::string actuator_state_sub_name;
        nhp_.param("actuator_state_sub_name", actuator_state_sub_name, std::string("joint_state"));
        actuator_state_sub_ = nh_.subscribe(actuator_state_sub_name, 1, &EndEffectorIKSolverCore::actuatorStateCallback, this);
        end_effector_ik_service_ = nh_.advertiseService("end_effector_ik", &EndEffectorIKSolverCore::endEffectorIkCallback, this);
        env_collision_sub_ = nh_.subscribe("/env_collision", 1, &EndEffectorIKSolverCore::envCollision, this);
      }
  }

bool EndEffectorIKSolverCore::endEffectorIkCallback(differential_kinematics::TargetPose::Request  &req,
                                                    differential_kinematics::TargetPose::Response &res)
{
  /* stop rosnode: joint_state_publisher, and publisher from this node instead */
  std::string actuator_state_publisher_node_name = actuator_state_sub_.getTopic().substr(0, actuator_state_sub_.getTopic().find("/joint")) + std::string("/joint_state_publisher_");
  std::string command_string = std::string("rosnode kill ") + actuator_state_publisher_node_name.c_str();
  system(command_string.c_str());

  collision_avoidance_ = req.collision_avoidance;
  /* start IK */
  tf::Quaternion q; q.setRPY(req.target_rot.x, req.target_rot.y, req.target_rot.z);
  tf::Transform target_ee_pose(q, tf::Vector3(req.target_pos.x, req.target_pos.y, req.target_pos.z));

  tf::Transform init_root_pose;
  init_root_pose.setIdentity();

  /* set the end effector */
  setEndEffectorPose(std::string("link") + std::to_string(planner_core_ptr_->getRobotModelPtr()->getRotorNum()),
                     tf::Transform(tf::createIdentityQuaternion(),
                                   tf::Vector3(planner_core_ptr_->getRobotModelPtr()->getLinkLength(), 0, 0)));

  if(!inverseKinematics(target_ee_pose, init_actuator_vector_, init_root_pose,
                        req.orientation, req.full_body, req.tran_free_axis, req.rot_free_axis,
                        req.collision_avoidance, req.debug))
    return false;

  return true;
}

void EndEffectorIKSolverCore::setEndEffectorPose(std::string parent_seg, tf::Transform pose)
{
  parent_seg_ = parent_seg;
  end_effector_relative_pose_ = pose;
}

void EndEffectorIKSolverCore::envCollision(const visualization_msgs::MarkerArrayConstPtr& env_msg)
{
  setCollision(*env_msg);
}

bool EndEffectorIKSolverCore::inverseKinematics(const tf::Transform& target_ee_pose, const sensor_msgs::JointState& init_actuator_vector, const tf::Transform& init_root_pose, bool orientation, bool full_body, std::string tran_free_axis, std::string rot_free_axis, bool collision_avoidance, bool debug)
{
  /* reset path */
  path_.resize(0);
  /* important: link1(root) should be base link */
  planner_core_ptr_->getRobotModelPtr()->setBaselinkName(std::string("link1"));

  /* declare the differential kinemtiacs const */
  pluginlib::ClassLoader<cost::Base>  cost_plugin_loader("differential_kinematics", "differential_kinematics::cost::Base");
  CostContainer cost_container;

  /* 1.  state_vel */
  cost_container.push_back(cost_plugin_loader.createInstance("differential_kinematics_cost/state_vel"));

  cost_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_cost/state_vel", orientation, full_body);

  /* 2. end-effector cartesian error constraint (cost) */
  cost_container.push_back(cost_plugin_loader.createInstance("differential_kinematics_cost/cartesian_constraint"));
  cost_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_cost/cartesian_constraint", orientation, full_body);

  /* special process: definition of end coords */
  KDL::Frame ee_frame; tf::poseTFToKDL(end_effector_relative_pose_, ee_frame);
  reinterpret_cast<cost::CartersianConstraint*>(cost_container.back().get())->updateChain("root", parent_seg_, KDL::Segment(std::string("end_effector"), KDL::Joint(KDL::Joint::None), ee_frame));
  //boost::dynamic_pointer_cast<cost::CartersianConstraint>(cost_container.back())->updateChain("root", std::string("link") + std::to_string(rotor_num), KDL::Segment(std::string("end_effector"), KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(planner_core_ptr_->getRobotModelPtr()->getLinkLength(), 0, 0))));
  /* TODO: following end effector is not valid for full-body ik solution, can not figure the reason */
  //boost::dynamic_pointer_cast<cost::CartersianConstraint>(cost_container.back())->updateChain("root", std::string("link3"), KDL::Segment(std::string("end_effector"), KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(planner_core_ptr_->getRobotModelPtr()->getLinkLength(), 0, 0))));


  //boost::dynamic_pointer_cast<cost::CartersianConstraint>(cost_container.back())->updateTargetFrame(target_ee_pose_);
  target_ee_pose_ = target_ee_pose;
  reinterpret_cast<cost::CartersianConstraint*>(cost_container.back().get())->updateTargetFrame(target_ee_pose);

  /* set free axis */
  std::vector<int> free_axis_list(0);
  if(tran_free_axis.find("x") != std::string::npos) free_axis_list.push_back(MaskAxis::TRAN_X);
  if(tran_free_axis.find("y") != std::string::npos) free_axis_list.push_back(MaskAxis::TRAN_Y);
  if(tran_free_axis.find("z") != std::string::npos) free_axis_list.push_back(MaskAxis::TRAN_Z);
  if(rot_free_axis == std::string("x")) free_axis_list.push_back(MaskAxis::ROT_X);
  if(rot_free_axis == std::string("y")) free_axis_list.push_back(MaskAxis::ROT_Y);
  if(rot_free_axis == std::string("z")) free_axis_list.push_back(MaskAxis::ROT_Z);

  if(free_axis_list.size() > 0)
    reinterpret_cast<cost::CartersianConstraint*>(cost_container.back().get())->setFreeAxis(free_axis_list);

  /* declare the differential kinemtiacs constraint */
  pluginlib::ClassLoader<constraint::Base>  constraint_plugin_loader("differential_kinematics", "differential_kinematics::constraint::Base");
  ConstraintContainer constraint_container;
  /* 1.  state_limit */
  constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/state_limit"));
  constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/state_limit", orientation, full_body);
  /* 2.  stability */
  constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/stability"));
  constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/stability", orientation, full_body);
  /* 3. cog motion */
  constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/cog_motion"));
  constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/cog_motion", orientation, full_body);
  /* 4. collision avoidance */
  if(collision_avoidance)
    {
      constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/collision_avoidance"));
      constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/collision_avoidance", orientation, full_body);
      //boost::dynamic_pointer_cast<constraint::CollisionAvoidance>(constraint_container.back())->setEnv(env_collision_);
      reinterpret_cast<constraint::CollisionAvoidance*>(constraint_container.back().get())->setEnv(env_collision_);
    }

  /* reset the init joint(actuator) state the init root pose for planner */
  planner_core_ptr_->setTargetRootPose(init_root_pose);
  planner_core_ptr_->setTargetActuatorVector(init_actuator_vector);

  /* start the planning */
  if(planner_core_ptr_->solver(cost_container, constraint_container, debug))
    {
      /* revert to the correct base link ( which is not root_link = link1), to be suitable for the control system */
      robot_model_ptr_->setBaselinkName(baselink_name_);

      for(int index = 0; index < planner_core_ptr_->getRootPoseSequence().size(); index++)
        {
          /* insert the result discrete path */
          geometry_msgs::Pose root_pose;
          tf::poseTFToMsg(planner_core_ptr_->getRootPoseSequence().at(index), root_pose);
          path_.push_back(MultilinkState(robot_model_ptr_, root_pose, planner_core_ptr_->getActuatorStateSequence().at(index)));
        }

      return true;
    }

  /* cannot solver */
  return false;
}

void EndEffectorIKSolverCore::motionFunc()
  {
    ros::Time now_time = ros::Time::now();
    br_.sendTransform(tf::StampedTransform(target_ee_pose_, now_time, "world", "target_ee"));

    tf::Transform end_link_ee_tf;
    end_link_ee_tf.setIdentity();
    end_link_ee_tf.setOrigin(tf::Vector3(planner_core_ptr_->getRobotModelPtr()->getLinkLength(), 0, 0));
    int rotor_num = planner_core_ptr_->getRobotModelPtr()->getRotorNum();
    br_.sendTransform(tf::StampedTransform(end_link_ee_tf, now_time, std::string("link") + std::to_string(rotor_num), "ee"));
  }

void EndEffectorIKSolverCore::actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  if(init_actuator_vector_.name.empty())
    {
      ROS_ERROR("get init actuator vector");
      init_actuator_vector_ = *state; /* only use the first angle vector */
    }
}

