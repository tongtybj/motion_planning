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

#include <differential_kinematics/constraint/base_plugin.h>

#include <tf_conversions/tf_kdl.h>
#include <sensor_msgs/JointState.h>
#include <hydrus/hydrus_robot_model.h>

namespace differential_kinematics
{
  namespace constraint
  {
    class Stability :public Base
    {
    public:
      Stability():
        fc_t_min_(1e6), fc_rp_min_(1e6)
      {}
      ~Stability(){}

      void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                              boost::shared_ptr<differential_kinematics::Planner> planner, std::string constraint_name,
                              bool orientation, bool full_body)
      {
        Base::initialize(nh, nhp, planner, constraint_name, orientation, full_body);

        nc_ = rotor_num_;

        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());

        getParam<bool>("check_fc_t", check_fc_t_, true);
        getParam<double>("fc_t_min_thre", fc_t_min_thre_, robot_model->getFeasibleControlTMinThre());
        getParam<double>("fc_t_dist_decrease_vel_thre", fc_t_dist_decrease_vel_thre_, -0.1);

        getParam<double>("fc_rp_min_thre", fc_rp_min_thre_, robot_model->getFeasibleControlRollPitchMinThre());
        getParam<double>("fc_rp_dist_decrease_vel_thre", fc_rp_dist_decrease_vel_thre_, -0.1);
        getParam<double>("fc_rp_dist_constraint_range", fc_rp_dist_constraint_range_, 0.2);
        getParam<double>("fc_rp_dist_forbidden_range", fc_rp_dist_forbidden_range_, 0.05);

        if(check_fc_t_) nc_ += rotor_num_;
      }

      bool getConstraint(Eigen::MatrixXd& A, Eigen::VectorXd& lb, Eigen::VectorXd& ub, bool debug = false)
      {
        const auto robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(planner_->getRobotModelPtr());

        A = Eigen::MatrixXd::Zero(nc_, robot_model->getLinkJointIndices().size() + 6);
        lb = Eigen::VectorXd::Constant(nc_, -0.1);
        ub = Eigen::VectorXd::Constant(nc_, 1e6);


        const auto& fc_rp_dists_jacobian = robot_model->getFeasibleControlRollPitchDistsJacobian();
        Eigen::VectorXd fc_rp_dists = robot_model->getFeasibleControlRollPitchDists();
        for(int i = 0; i < rotor_num_; i++)
          {
            Eigen::MatrixXd::Index index;
            double rp_min = fc_rp_dists.minCoeff(&index);
            if(i == 0)
              {
                if(rp_min < fc_rp_min_)
                  {
                    fc_rp_min_ = rp_min;
                  }
              }
            A.row(i) = fc_rp_dists_jacobian.row(index);
            lb(i) = damplingBound(rp_min - fc_rp_min_thre_,
                                  fc_rp_dist_decrease_vel_thre_,
                                  fc_rp_dist_constraint_range_,
                                  fc_rp_dist_forbidden_range_);

            fc_rp_dists(index) = 1e6; // reset
          }

        if(debug)
          {
            std::cout << "constraint (" << constraint_name_.c_str()  << "): feasible control roll pitch convex distances: \n" << robot_model->getFeasibleControlRollPitchDists().transpose() << std::endl;
            std::cout << "constraint (" << constraint_name_.c_str()  << "): fc_rp_jacobian: \n" << fc_rp_dists_jacobian << std::endl;
          }

        if(check_fc_t_)
          {
            Eigen::VectorXd fc_t_dists = robot_model->getFeasibleControlTDists();
            const auto& fc_t_dists_jacobian = robot_model->getFeasibleControlTDistsJacobian();

            // only choose rotor_num component
            for(int i = 0; i < rotor_num_; i++)
              {
                Eigen::MatrixXd::Index index;
                double t_min = fc_t_dists.minCoeff(&index);
                if(i == 0)
                  {
                    if(t_min < fc_t_min_)
                      {
                        fc_t_min_ = t_min;
                      }
                  }
                A.row(i + rotor_num_) = fc_t_dists_jacobian.row(index);
                double diff = fc_t_min_thre_ - t_min;
                lb(i + rotor_num_) =  diff < fc_t_dist_decrease_vel_thre_? fc_t_dist_decrease_vel_thre_:diff;
                fc_t_dists(index) = 1e6; // reset
              }

            if(debug)
              {
                std::cout << "constraint (" << constraint_name_.c_str()  << "): feasible control torque convex distances: \n" << robot_model->getFeasibleControlTDists().transpose() << std::endl;
                std::cout << "constraint (" << constraint_name_.c_str()  << "): fc_t_dists_jacobian: \n" << fc_t_dists_jacobian << std::endl;
              }
          }


        if(debug)
          {
            std::cout << "constraint (" << constraint_name_.c_str()  << "): matrix A: \n" << A << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", lb: \n" << lb.transpose() << std::endl;
            std::cout << "constraint name: " << constraint_name_ << ", ub: \n" << ub.transpose() << std::endl;
          }
        return true;
      }

      void result()
      {
        std::cout << constraint_name_
                  << "min fc_rp_min_: " << fc_rp_min_
                  << "; fc_rp_min_rp_position_margin_: " << fc_rp_min_rp_position_margin_  << std::endl;
        if(check_fc_t_)
          {
            std::cout << constraint_name_
                      << "min fc_t_min_: " << fc_t_min_
                      << "; fc_t_min_rp_position_margin_: " << fc_t_min_rp_position_margin_  << std::endl;
          }
      }

      bool directConstraint(){return false;}

    protected:

      bool check_fc_t_;
      double fc_t_min_thre_;
      double fc_t_dist_decrease_vel_thre_;
      double fc_rp_min_thre_;
      double fc_rp_dist_decrease_vel_thre_;
      double fc_rp_dist_constraint_range_;
      double fc_rp_dist_forbidden_range_;

      double fc_t_min_;
      double fc_rp_min_;
      double fc_t_min_rp_position_margin_;
      double fc_rp_min_rp_position_margin_;
    };
  };
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(differential_kinematics::constraint::Stability, differential_kinematics::constraint::Base);

