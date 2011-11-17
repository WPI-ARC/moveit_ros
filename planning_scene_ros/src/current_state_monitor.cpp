/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "planning_scene_ros/current_state_monitor.h"

planning_scene_ros::CurrentStateMonitor::CurrentStateMonitor(const planning_models::KinematicModelConstPtr &kmodel, tf::Transformer *tf) :
    tf_(tf), kmodel_(kmodel), kstate_(kmodel), root_(kstate_.getJointState(kmodel->getRoot()->getName())), state_monitor_started_(false)
{
}

planning_models::KinematicStatePtr planning_scene_ros::CurrentStateMonitor::getCurrentState(void) const
{
    boost::mutex::scoped_lock slock(state_update_lock_);
    planning_models::KinematicState *result = new planning_models::KinematicState(kstate_);
    return planning_models::KinematicStatePtr(result);
}

std::map<std::string, double> planning_scene_ros::CurrentStateMonitor::getCurrentStateValues(void) const
{
    std::map<std::string, double> m;
    boost::mutex::scoped_lock slock(state_update_lock_);
    kstate_.getStateValues(m);
    return m;
}

void planning_scene_ros::CurrentStateMonitor::setOnStateUpdateCallback(const JointStateUpdateCallback &callback)
{
    on_state_update_callback_ = callback;
}

void planning_scene_ros::CurrentStateMonitor::startStateMonitor(void)
{
    if (!state_monitor_started_ && kmodel_)
    {
        joint_time_.clear();
        joint_state_subscriber_ = nh_.subscribe("joint_states", 25, &CurrentStateMonitor::jointStateCallback, this);
        state_monitor_started_ = true;
        ROS_DEBUG("Listening to joint states");
    }
}

void planning_scene_ros::CurrentStateMonitor::stopStateMonitor(void)
{
    if (state_monitor_started_)
    {
        joint_state_subscriber_.shutdown();
        ROS_DEBUG("No longer listening o joint states");
        state_monitor_started_ = false;
    }
}

bool planning_scene_ros::CurrentStateMonitor::haveCompleteState(void) const
{
    bool result = true;
    const std::vector<std::string> &dof = kmodel_->getActiveDOFNames();
    boost::mutex::scoped_lock slock(state_update_lock_);
    for (std::size_t i = 0 ; i < dof.size() ; ++i)
        if (joint_time_.find(dof[i]) == joint_time_.end())
        {
            ROS_DEBUG("Joint variable '%s' has never been updated", dof[i].c_str());
            result = false;
        }
    return result;
}

bool planning_scene_ros::CurrentStateMonitor::haveCompleteState(const ros::Duration &age) const
{
    bool result = true;
    const std::vector<std::string> &dof = kmodel_->getActiveDOFNames();
    ros::Time now = ros::Time::now();
    ros::Time old = now - age;
    boost::mutex::scoped_lock slock(state_update_lock_);
    for (std::size_t i = 0 ; i < dof.size() ; ++i)
    {
        std::map<std::string, ros::Time>::const_iterator it = joint_time_.find(dof[i]);
        if (it == joint_time_.end())
        {
            ROS_DEBUG("Joint variable '%s' has never been updated", dof[i].c_str());
            result = false;
        }
        else
            if (it->second < old)
            {
                ROS_DEBUG("Joint variable '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                          dof[i].c_str(), (now - it->second).toSec(), age.toSec());
                result = false;
            }
    }
    return result;
}

void planning_scene_ros::CurrentStateMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr &joint_state)
{
    if (joint_state->name.size() != joint_state->position.size())
    {
        ROS_ERROR("State monitor received invalid joint state");
        return;
    }

    // read the received values, and update their time stamps
    std::size_t n = joint_state->name.size();
    std::map<std::string, double> joint_state_map;
    for (std::size_t i = 0 ; i < n ; ++i)
    {
        joint_state_map[joint_state->name[i]] = joint_state->position[i];
        joint_time_[joint_state->name[i]] = joint_state->header.stamp;
    }
    bool set_map_values = true;

    // read root transform, if needed
    if (root_->getType() == planning_models::KinematicModel::JointModel::PLANAR ||
        root_->getType() == planning_models::KinematicModel::JointModel::FLOATING)
    {
        const std::string &child_frame = root_->getJointModel()->getChildLinkModel()->getName();
        const std::string &parent_frame = kmodel_->getModelFrame();

        std::string err;
        ros::Time tm;
        tf::StampedTransform transf;
        bool ok = false;
        if (tf_->getLatestCommonTime(parent_frame, child_frame, tm, &err) == tf::NO_ERROR)
        {
            try
            {
                tf_->lookupTransform(parent_frame, child_frame, tm, transf);
                ok = true;
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", parent_frame.c_str(), child_frame.c_str(), ex.what());
            }
        }
        else
            ROS_DEBUG("Unable to lookup transform from %s to %s: no common time.", parent_frame.c_str(), child_frame.c_str());
        if (ok)
        {
            const std::vector<std::string> &vars = root_->getJointModel()->getVariableNames();
            for (std::size_t j = 0; j < vars.size() ; ++j)
                joint_time_[vars[j]] = tm;
            set_map_values = false;
            boost::mutex::scoped_lock slock(state_update_lock_);
            root_->setVariableValues(transf);
            kstate_.setStateValues(joint_state_map);
        }
    }

    if (set_map_values)
    {
        boost::mutex::scoped_lock slock(state_update_lock_);
        kstate_.setStateValues(joint_state_map);
    }

    // callback, if needed
    if (on_state_update_callback_)
        on_state_update_callback_(joint_state);
}