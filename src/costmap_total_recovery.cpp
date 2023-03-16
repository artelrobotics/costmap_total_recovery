/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <costmap_total_recovery/costmap_total_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <boost/pointer_cast.hpp>
#include <vector>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(costmap_total_recovery::CostmapTotalRecovery, nav_core::RecoveryBehavior)

using costmap_2d::NO_INFORMATION;

namespace costmap_total_recovery {
CostmapTotalRecovery::CostmapTotalRecovery(): global_costmap_(NULL), local_costmap_(NULL),
  tf_(NULL), initialized_(false) {}

void CostmapTotalRecovery::initialize(std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

void CostmapTotalRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the CostmapTotalRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  else
  {
    ROS_WARN("Running CostmapTotalRecovery");
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_local(*(local_costmap_->getCostmap()->getMutex()));
    local_costmap_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_global(*(global_costmap_->getCostmap()->getMutex()));
    global_costmap_->resetLayers();
  }
}

};
