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

#ifndef END_EFFECTOR_IK_SOLVER_CORE_H
#define END_EFFECTOR_IK_SOLVER_CORE_H

/* special cost plugin for cartesian constraint */
#include <differential_kinematics/cost/cartesian_constraint.h>
/* special constraint plugin for collision avoidance */
#include <differential_kinematics/constraint/collision_avoidance.h>

#include <differential_kinematics/planner_core.h>

#include <pluginlib/class_loader.h>
/* rosservice for target end-effector pose */
#include <visualization_msgs/MarkerArray.h>
#include <differential_kinematics/TargetPose.h>
#include <aerial_motion_planning_msgs/multilink_state.h>

using namespace differential_kinematics;

class EndEffectorIKSolverCore
{
public:
  EndEffectorIKSolverCore(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<HydrusRobotModel> robot_model_ptr, bool simulation);
  ~EndEffectorIKSolverCore(){}

  const std::string getParentSegName() const {return parent_seg_;}
  const tf::Transform getEndEffectorRelativePose() const {return end_effector_relative_pose_;}
  const std::vector<MultilinkState>& getPathConst() const {return path_;}

  void setEndEffectorPose(std::string parent_seg, tf::Transform pose);
  void setCollision(const visualization_msgs::MarkerArray& env_collision)
  {
    env_collision_ = env_collision;
  }

  bool inverseKinematics(const tf::Transform& target_ee_pose, const sensor_msgs::JointState& init_actuator_vector, const tf::Transform& init_root_pose, bool orientation, bool full_body, std::string tran_free_axis, std::string rot_free_axis, bool collision_avoidance, bool debug);

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::ServiceServer end_effector_ik_service_;
  ros::Subscriber actuator_state_sub_;
  ros::Subscriber env_collision_sub_;
  tf::TransformBroadcaster br_;

  boost::shared_ptr<HydrusRobotModel> robot_model_ptr_;
  std::string baselink_name_;
  std::string parent_seg_;
  tf::Transform end_effector_relative_pose_;

  boost::shared_ptr<Planner> planner_core_ptr_;
  std::vector<MultilinkState> path_;
  tf::Transform target_ee_pose_;
  sensor_msgs::JointState init_actuator_vector_;

  /* collision avoidance */
  bool collision_avoidance_;
  visualization_msgs::MarkerArray env_collision_;

  void actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state);
  bool endEffectorIkCallback(differential_kinematics::TargetPose::Request  &req,
                             differential_kinematics::TargetPose::Response &res);

  void envCollision(const visualization_msgs::MarkerArrayConstPtr& env_msg);
  void motionFunc();

};

#endif
