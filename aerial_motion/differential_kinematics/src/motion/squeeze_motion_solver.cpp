// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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
#include <differential_kinematics/motion/squeeze_motion_solver.h>

using namespace differential_kinematics;
using robot_model = TransformController;
using CostContainer = std::vector<boost::shared_ptr<cost::Base<Planner> > >;
using ConstraintContainer = std::vector<boost::shared_ptr<constraint::Base<Planner> > >;

DifferentialKinematicsSolver::DifferentialKinematicsSolver(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<robot_model> robot_model_ptr): nh_(nh), nhp_(nhp), robot_model_ptr_(robot_model_ptr), path_(0)
{
    /* setup the planner core */
    planner_core_ptr_ = boost::shared_ptr<Planner> (new Planner(nh, nhp, robot_model_ptr_));
    /* important: set the link1(root) as the base link */
    planner_core_ptr_->getRobotModelPtr()->setBaselink(std::string("link1"));
    //planner_core_ptr_->registerMotionFunc(std::bind(&DifferentialKinematicsSolver::motionFunc, this));
    planner_core_ptr_->registerUpdateFunc(std::bind(&DifferentialKinematicsSolver::updatePinchPoint, this));

    /* base vars */
    phase_ = CASE1;
    reference_point_ratio_ = 1.0;

    nhp_.param("delta_pinch_length", delta_pinch_length_, 0.02); // [m]
    nhp_.param("debug", debug_, true);

    /* init root pose & init actuator state */
    /* setup robot joint model */
    sensor_msgs::JointState actuator_state;
    for (auto itr: robot_model_ptr_->getActuatorMap())
      {
        actuator_state.name.push_back(itr.first);
        actuator_state.position.push_back(0);
      }
    robot_model_ptr_->setActuatorJointMap(actuator_state); // get joint index from get actuator map

    start_state_.setRootPose(geometry_msgs::Pose());
    goal_state_.setRootPose(geometry_msgs::Pose());
    start_state_.setActuatorState(actuator_state);
    goal_state_.setActuatorState(actuator_state);
    nhp_.param("start_state_x", start_state_.getRootPoseNonConst().position.x, 0.0);
    nhp_.param("start_state_y", start_state_.getRootPoseNonConst().position.y, 0.5);
    nhp_.param("start_state_z", start_state_.getRootPoseNonConst().position.z, 0.0);
    double r, p, y;
    nhp_.param("start_state_roll", r, 0.0);
    nhp_.param("start_state_pitch", p, 0.0);
    nhp_.param("start_state_yaw", y, 0.0);
    start_state_.getRootPoseNonConst().orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);

    nhp_.param("goal_state_x", goal_state_.getRootPoseNonConst().position.x, 0.0);
    nhp_.param("goal_state_y", goal_state_.getRootPoseNonConst().position.y, 0.5);
    nhp_.param("goal_state_z", goal_state_.getRootPoseNonConst().position.z, 0.0);
    nhp_.param("goal_state_roll", r, 0.0);
    nhp_.param("goal_state_pitch", p, 0.0);
    nhp_.param("goal_state_yaw", y, 0.0);
    goal_state_.getRootPoseNonConst().orientation = tf::createQuaternionMsgFromRollPitchYaw(r,p,y);

    for(int i = 0; i < robot_model_ptr_->getActuatorJointMap().size(); i++)
      {
        int joint_no = robot_model_ptr_->getActuatorJointMap().at(i);
        nhp_.param(std::string("start_") + start_state_.getActuatorStateNonConst().name.at(joint_no), start_state_.getActuatorStateNonConst().position.at(joint_no), 0.0);
        nhp_.param(std::string("goal_") + goal_state_.getActuatorStateNonConst().name.at(joint_no), goal_state_.getActuatorStateNonConst().position.at(joint_no), 0.0);
      }

    ROS_WARN("model: %d", planner_core_ptr_->getMultilinkType());

    if(planner_core_ptr_->getMultilinkType() == motion_type::SE2)
      {
        ROS_WARN("correct model");
        /* setup env */
        double openning_width, env_width, env_length;
        double wall_thinkness = 0.05;
        nhp_.param("openning_width", openning_width, 0.8);
        nhp_.param("env_width", env_width, 4.0);
        nhp_.param("env_length", env_length, 6.0);
        /* openning side wall(s) */
        visualization_msgs::Marker wall;
        wall.type = visualization_msgs::Marker::CUBE;
        wall.action = visualization_msgs::Marker::ADD;
        wall.header.frame_id = "/world";
        wall.color.g = 1;
        wall.color.a = 1;
        wall.pose.orientation.w = 1;
        wall.scale.z = 2;

        wall.id = 1;
        wall.pose.position.x = 0;
        wall.pose.position.y = -env_width / 2;
        wall.pose.position.z = 0;
        wall.scale.x = env_length;
        wall.scale.y = wall_thinkness;
        env_collision_.markers.push_back(wall);

        wall.id = 2;
        wall.pose.position.y = env_width / 2;
        env_collision_.markers.push_back(wall);

        wall.id = 3;
        wall.pose.position.x = 0;
        wall.pose.position.y = env_width / 4 + openning_width / 4;
        wall.pose.position.z = 0;
        wall.scale.x = wall_thinkness;
        wall.scale.y = env_width / 2 - openning_width / 2;
        env_collision_.markers.push_back(wall);

        wall.id = 4;
        wall.pose.position.y = -env_width / 4 - openning_width / 4;
        env_collision_.markers.push_back(wall);

        openning_center_frame_.setOrigin(tf::Vector3(0, 0, 0));
        openning_center_frame_.setRotation(tf::createQuaternionFromRPY(0, M_PI / 2, 0)); // the head should be z axis
      }
    else if(planner_core_ptr_->getMultilinkType() == motion_type::SE3)
      {
        /* setup env */
        double openning_width, openning_height, env_width, env_length;
        double wall_thinkness = 0.05;
        nhp_.param("openning_width", openning_width, 0.8);
        nhp_.param("openning_height", openning_height, 0.8);
        nhp_.param("env_width", env_width, 6.0);
        /* openning side wall(s) */
        visualization_msgs::Marker wall;
        wall.type = visualization_msgs::Marker::CUBE;
        wall.action = visualization_msgs::Marker::ADD;
        wall.header.frame_id = "/world";
        wall.color.g = 1;
        wall.color.a = 1;
        wall.pose.orientation.w = 1;
        wall.scale.z = 0.05;
        wall.pose.position.z = openning_height;

        wall.id = 1;
        wall.pose.position.x = env_width / 4 + openning_width / 4;
        wall.pose.position.y = 0;
        wall.scale.x = env_width / 2 - openning_width / 2;
        wall.scale.y = env_width;
        env_collision_.markers.push_back(wall);

        wall.id = 2;
        wall.pose.position.x = -wall.pose.position.x;
        env_collision_.markers.push_back(wall);

        wall.id = 3;
        wall.pose.position.x = 0;
        wall.pose.position.y = env_width / 4 + openning_width / 4;
        wall.scale.x = openning_width;
        wall.scale.y = env_width / 2 - openning_width / 2;
        env_collision_.markers.push_back(wall);

        wall.id = 4;
        wall.pose.position.y = -wall.pose.position.y;
        env_collision_.markers.push_back(wall);

        wall.id = 5;
        wall.pose.position.x = 0;
        wall.pose.position.y = 0;
        wall.scale.x = env_width;
        wall.scale.y = env_width;
        wall.pose.position.z = openning_height + 0.45; // + 0.5
        env_collision_.markers.push_back(wall);

        openning_center_frame_.setOrigin(tf::Vector3(0, 0, openning_height));
        openning_center_frame_.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
      }
    env_collision_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/env_collision", 1);
}


bool DifferentialKinematicsSolver::plan(bool debug)
{
  /* reset */
  path_.clear();

  /* declare the differential kinemtiacs const */
  pluginlib::ClassLoader<cost::Base<Planner> >  cost_plugin_loader("differential_kinematics", "differential_kinematics::cost::Base<differential_kinematics::Planner>");
  CostContainer cost_container;
  /* 1. statevel */
  cost_container.push_back(cost_plugin_loader.createInstance("differential_kinematics_cost/state_vel"));
  cost_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_cost/state_vel", true /* orientation */, true /* full_body */);
  /* 2. reference point cartesian error constraint (cost) */
  cost_container.push_back(cost_plugin_loader.createInstance("differential_kinematics_cost/cartesian_constraint"));
  cost_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_cost/cartesian_constraint", false /* orientation */, true /* full_body */);
  cartersian_constraint_ = boost::dynamic_pointer_cast<cost::CartersianConstraint<Planner> >(cost_container.back());
  /* defualt reference point is the openning center */
  tf::Transform target_frame = openning_center_frame_;
  target_frame.setOrigin(target_frame.getOrigin() + openning_center_frame_.getBasis() * tf::Vector3(0, 0, delta_pinch_length_));
  cartersian_constraint_->updateTargetFrame(target_frame);
  ROS_WARN("target frame:[%f, %f, %f]", target_frame.getOrigin().x(),
           target_frame.getOrigin().y(), target_frame.getOrigin().z());

  /* declare the differential kinemtiacs constraint */
  pluginlib::ClassLoader<constraint::Base<Planner> >  constraint_plugin_loader("differential_kinematics", "differential_kinematics::constraint::Base<differential_kinematics::Planner>");
  ConstraintContainer constraint_container;
  /* 1.  state_limit */
  constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/state_limit"));
  constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/state_limit", true /* orientation */, true /* full_body */);
  /* 2.  stability */
  constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/stability"));
  constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/stability", true /* orientation */, true /* full_body */);
  /* 3. collision avoidance */
  constraint_container.push_back(constraint_plugin_loader.createInstance("differential_kinematics_constraint/collision_avoidance"));
  constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, "differential_kinematics_constraint/collision_avoidance", true /* orientation */, true /* full_body */);
  boost::dynamic_pointer_cast<constraint::CollisionAvoidance<Planner> >(constraint_container.back())->setEnv(env_collision_);

  /* 4. additional plugins for cost and constraint, if necessary */
  auto pattern_match = [&](std::string &pl, std::string &pl_candidate) -> bool
    {
      int cmp = fnmatch(pl.c_str(), pl_candidate.c_str(), FNM_CASEFOLD);
      if (cmp == 0)
        return true;
      else if (cmp != FNM_NOMATCH) {
        // never see that, i think that it is fatal error.
        ROS_FATAL("Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
                  pl.c_str(), pl_candidate.c_str(), cmp);
      }
      return false;
    };

  ros::V_string additional_constraint_list{};
  nhp_.getParam("additional_constraint_list", additional_constraint_list);
  for (auto &plugin_name : additional_constraint_list)
    {
      for (auto &name : constraint_plugin_loader.getDeclaredClasses())
        {
          if(!pattern_match(plugin_name, name)) continue;

          constraint_container.push_back(constraint_plugin_loader.createInstance(name));
          constraint_container.back()->initialize(nh_, nhp_, planner_core_ptr_, name, true, true);
          break;
        }
    }

  /* reset the init joint(actuator) state the init root pose for planner */
  tf::Transform root_pose;
  tf::poseMsgToTF(start_state_.getRootPoseConst(), root_pose);
  planner_core_ptr_->setInitRootPose(root_pose);
  planner_core_ptr_->setInitActuatorPose(start_state_.getActuatorStateConst());

  /* init the pinch point, shouch be the end point of end link */
  updatePinchPoint();

  /* start the planning */
   if(planner_core_ptr_->solver(cost_container, constraint_container, debug))
   {
     for(int index = 0; index < planner_core_ptr_->getPathSize(); index++ )
       {
         MultilinkState robot_state;
         geometry_msgs::Pose root_pose;
         tf::poseTFToMsg(planner_core_ptr_->getRootPoseSequence().at(index), root_pose);
         robot_state.setRootPose(root_pose);
         robot_state.setActuatorState(planner_core_ptr_->getActuatorStateSequence().at(index));
         robot_state.targetRootPose2TargetBaselinkPose(robot_model_ptr_);
         path_.push_back(robot_state);
       }
     return true;
   }
   else return false;
}

bool DifferentialKinematicsSolver::updatePinchPoint()
{
  /* case 1: total not pass: use the head */
  /* case 2: half pass: calcualte the pass point */
  /* case 3: final phase: the root link is close to the openning_center gap => finish */
  KDL::TreeFkSolverPos_recursive fk_solver(robot_model_ptr_->getModelTree());
  KDL::JntArray joint_positions(robot_model_ptr_->getModelTree().getNrOfJoints());

  /* case3 */
  if((openning_center_frame_.inverse() * planner_core_ptr_->getTargetRootPose()).getOrigin().z() > -0.01)
    {
      if(phase_ < CASE2_2)
        {
          ROS_ERROR("CurrentPhase: %d, wrong phase:%d, ", phase_, CASE1);
          return false;
        }

      if(debug_) ROS_INFO("case 3");
      /* convergence */
      ROS_WARN("complete passing regarding to the cartersian constraint");
      //cartersian_constraint_->updateTargetFrame(planner_core_ptr_->getTargetRootPose());
      cartersian_constraint_->updateChain("root", std::string("link1"), KDL::Segment(std::string("pinch_point"), KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(0, 0, 0))));
      return true;
    }

  sensor_msgs::JointState actuator_state = planner_core_ptr_->getTargetActuatorVector();
  /* considering the gimbal target angle */
  robot_model_ptr_->forwardKinematics(actuator_state);

  auto actuator_map = robot_model_ptr_->getActuatorMap();
  for(size_t i = 0; i < actuator_state.position.size(); i++)
    {
      auto itr = actuator_map.find(actuator_state.name.at(i));

      if(itr != actuator_map.end())  joint_positions(actuator_map.find(actuator_state.name.at(i))->second) = actuator_state.position.at(i);
    }

  tf::Transform previous_link_tf = planner_core_ptr_->getTargetRootPose();
  double previous_z;

  for(int index = 1; index <= robot_model_ptr_->getRotorNum(); index++)
    {
      std::stringstream ss;
      ss << index;
      KDL::Frame f_current_link;
      fk_solver.JntToCart(joint_positions, f_current_link, std::string("link") + ss.str());
      tf::Transform current_link_tf;
      tf::transformKDLToTF(f_current_link, current_link_tf);
      double current_z = (openning_center_frame_.inverse() * planner_core_ptr_->getTargetRootPose() * current_link_tf).getOrigin().z();
      if(current_z > 0)
        {
          /* case 2.2 */
          if(phase_ > CASE2_2 || phase_ == CASE1)
            {
              ROS_ERROR("CurrentPhase: %d, wrong phase:%d, ", phase_, CASE1);
              return false;
            }

          if(phase_ == CASE2_1) reference_point_ratio_ = 1;
          phase_ = CASE2_2;
          //if(debug_) ROS_INFO("case 2.2");
          assert(index > 1);
          std::stringstream ss;
          ss << index - 1 ;

          double new_ratio = fabs(previous_z) / (fabs(previous_z) + current_z);
          if(debug_) ROS_INFO("case 2.2, current_z: %f, previous_z: %f, new_ratio: %f, reference_point_ratio_: %f", current_z, previous_z, new_ratio, reference_point_ratio_);
          if(new_ratio < reference_point_ratio_) reference_point_ratio_ = new_ratio;

          cartersian_constraint_->updateChain("root", std::string("link") + ss.str(), KDL::Segment(std::string("pinch_point"), KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength() * new_ratio , 0, 0))));
          //if(debug_) ROS_WARN("under: link%d, z: %f; upper: link%d, z: %f", index -1, previous_z, index, current_z);
          return true;
        }
      previous_link_tf = current_link_tf;
      previous_z = current_z;
    }

  /* head of the robot */
  tf::Transform head_tf; head_tf.setOrigin(tf::Vector3(robot_model_ptr_->getLinkLength(), 0, 0));
  double head_z = (openning_center_frame_.inverse() * planner_core_ptr_->getTargetRootPose() * previous_link_tf * head_tf).getOrigin().z();

  //if(debug_) ROS_INFO("head_z: %f", head_z);
  std::stringstream ss;
  ss << robot_model_ptr_->getRotorNum();

  if (head_z < 0)  /* case 1 */
    {
      if(phase_ > CASE1)
        {
          ROS_ERROR("CurrentPhase: %d, wrong phase:%d, ", phase_, CASE1);
          return false;
        }
      if(debug_) ROS_INFO("case 1, the head does not pass");

      cartersian_constraint_->updateChain("root", std::string("link") + ss.str(), KDL::Segment(std::string("pinch_point"), KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength(), 0, 0))));
    }
  else /* case 2.1 */
    {
      if(phase_ > CASE2_1)
        {
          ROS_ERROR("CurrentPhase: %d, wrong phase:%d, ", phase_, CASE1);
          return false;
        }
      phase_ = CASE2_1;

      double new_ratio = fabs(previous_z) / (fabs(previous_z) + head_z);
      if(debug_) ROS_INFO("case 2.1, head_z: %f, previous_z: %f, new_ratio: %f, reference_point_ratio_: %f", head_z, previous_z, new_ratio, reference_point_ratio_);
      if(new_ratio < reference_point_ratio_) reference_point_ratio_ = new_ratio;

      cartersian_constraint_->updateChain("root", std::string("link") + ss.str(), KDL::Segment(std::string("pinch_point"), KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(robot_model_ptr_->getLinkLength() * reference_point_ratio_, 0, 0))));
    }

  return true;
}

void DifferentialKinematicsSolver::visualizeEnv()
{
  /* collsion publishment */
  env_collision_pub_.publish(env_collision_);
}