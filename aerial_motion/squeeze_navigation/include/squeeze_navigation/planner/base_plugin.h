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

#ifndef SQUEEZE_MOTION_PLANNER_PLUGIN_H
#define SQUEEZE_MOTION_PLANNER_PLUGIN_H


#include <ros/ros.h>
#include <dragon/transform_control.h>
#include <aerial_motion_planning_msgs/multilink_state.h>

namespace squeeze_motion_planner
{
  class Base
  {
  public:
    Base() {}
    ~Base() {}

    void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController> robot_model_ptr)
    {
      nh_ = nh;
      nhp_ = nhp;

      robot_model_ptr_ = robot_model_ptr;
    }

    virtual bool plan(bool debug = false) = 0;
    virtual bool loadPath() {return false; }
    virtual const std::vector<MultilinkState>& getPathConst() const = 0 ;
    virtual const MultilinkState& getStateConst(int index) const = 0 ;
    virtual void visualizeFunc() = 0;
    virtual void checkCollision(MultilinkState state) = 0;

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    boost::shared_ptr<TransformController> robot_model_ptr_;
  };
};

#endif
