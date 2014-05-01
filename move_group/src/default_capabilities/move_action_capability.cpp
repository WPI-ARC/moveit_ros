/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "move_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
//#include "cvxopt/cvxopt.h"

//display arrow include files
#include "shape_msgs/Mesh.h"
#include "shape_msgs/MeshTriangle.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/CollisionObject.h"
#include "visualization_msgs/Marker.h"

//controllerCB include files
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>


typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

move_group::MoveGroupMoveAction::MoveGroupMoveAction() :
    MoveGroupCapability("MoveAction"),
    move_state_(IDLE)
{  
}

void move_group::MoveGroupMoveAction::initialize()
{
    // start the move action server
    move_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::MoveGroupAction>(root_node_handle_, MOVE_ACTION,
                                                                                              boost::bind(&MoveGroupMoveAction::executeMoveCallback, this, _1), false));
    move_action_server_->registerPreemptCallback(boost::bind(&MoveGroupMoveAction::preemptMoveCallback, this));
    move_action_server_->start();
}

void move_group::MoveGroupMoveAction::executeMoveCallback(const moveit_msgs::MoveGroupGoalConstPtr& goal)
{
    setMoveState(PLANNING);
    context_->planning_scene_monitor_->updateFrameTransforms();

    moveit_msgs::MoveGroupResult action_res;
    if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
    {
        if (!goal->planning_options.plan_only)
            ROS_WARN("This instance of MoveGroup is not allowed to execute trajectories but the goal request has plan_only set to false. Only a motion plan will be computed anyway.");
        executeMoveCallback_PlanOnly(goal, action_res);
    }
    else
        executeMoveCallback_PlanAndExecute(goal, action_res);

    bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
    std::string response = getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
    if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        move_action_server_->setSucceeded(action_res, response);
    else
    {
        if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
            move_action_server_->setPreempted(action_res, response);
        else
            move_action_server_->setAborted(action_res, response);
    }

    setMoveState(IDLE);
}

void move_group::MoveGroupMoveAction::executeMoveCallback_PlanAndExecute(const moveit_msgs::MoveGroupGoalConstPtr& goal, moveit_msgs::MoveGroupResult &action_res)
{  
    ROS_INFO("Combined planning and execution request received for MoveGroup action. Forwarding to planning and execution pipeline.");

    if (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff))
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
        const robot_state::RobotState &current_state = lscene->getCurrentState();

        // check to see if the desired constraints are already met
        for (std::size_t i = 0 ; i < goal->request.goal_constraints.size() ; ++i)
            if (lscene->isStateConstrained(current_state, kinematic_constraints::mergeConstraints(goal->request.goal_constraints[i],
                                                                                                  goal->request.path_constraints)))
            {
                ROS_INFO("Goal constraints are already satisfied. No need to plan or execute any motions");
                action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
                return;
            }
    }

    plan_execution::PlanExecution::Options opt;

    const moveit_msgs::MotionPlanRequest &motion_plan_request = planning_scene::PlanningScene::isEmpty(goal->request.start_state) ?
                goal->request : clearRequestStartState(goal->request);
    const moveit_msgs::PlanningScene &planning_scene_diff = planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
                goal->planning_options.planning_scene_diff : clearSceneRobotState(goal->planning_options.planning_scene_diff);

    opt.replan_ = goal->planning_options.replan;
    opt.replan_attempts_ = goal->planning_options.replan_attempts;
    opt.replan_delay_ = goal->planning_options.replan_delay;
    opt.before_execution_callback_ = boost::bind(&MoveGroupMoveAction::startMoveExecutionCallback, this);

    opt.plan_callback_ = boost::bind(&MoveGroupMoveAction::planUsingPlanningPipeline, this, boost::cref(motion_plan_request), _1);
    if (goal->planning_options.look_around && context_->plan_with_sensing_)
    {
        opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_,
                                         goal->planning_options.look_around_attempts, goal->planning_options.max_safe_execution_cost);
        context_->plan_with_sensing_->setBeforeLookCallback(boost::bind(&MoveGroupMoveAction::startMoveLookCallback, this));
    }

    plan_execution::ExecutableMotionPlan plan;
    context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

    convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
    if (plan.executed_trajectory_)
        plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
    action_res.error_code = plan.error_code_;
}


/*************************************************controllerStateCB**************************************************************************/
double current_joint_position[7];
double current_joint_velocity[7];

void controllerStateCB(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    for (int joint_index = 0; joint_index < 7; joint_index++)
    {
        current_joint_position[joint_index]=msg->actual.positions[joint_index];
        current_joint_velocity[joint_index]=msg->actual.velocities[joint_index];
    }
}


void move_group::MoveGroupMoveAction::executeMoveCallback_PlanOnly(const moveit_msgs::MoveGroupGoalConstPtr& goal, moveit_msgs::MoveGroupResult &action_res)
{
    ROS_INFO("Planning request received for MoveGroup action. Forwarding to planning pipeline.");

    planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_); // lock the scene so that it does not modify the world representation while diff() is called
    const planning_scene::PlanningSceneConstPtr &the_scene = (planning_scene::PlanningScene::isEmpty(goal->planning_options.planning_scene_diff)) ?
                static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) : lscene->diff(goal->planning_options.planning_scene_diff);
    planning_interface::MotionPlanResponse res;

    /*************************************************DECLARATION for: show shortest distance ********************************************************/
    ros::NodeHandle handle;
    ros::Publisher vis_pub = handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    ros::Subscriber sub = handle.subscribe("r_arm_controller/state", 1000, &controllerStateCB);
    ros::Rate poll_rate(100);
    while(vis_pub.getNumSubscribers() == 0)
        poll_rate.sleep();



    /*************************************************Joint Velocity Constraints**************************************************************************/
    double velocity_constraints[] = {0.69, 0.69, 1.05, 1.1, 3, 3, 3};

    try
    {
        /*************************************************Get Robot State in a specific scene********************************************************/
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        robot_state::RobotState copied_state = the_scene->getCurrentState();
        /*************************************************Generate Path********************************************************/
        context_->planning_pipeline_->generatePlan(the_scene, goal->request, res);
        int path_size = res.trajectory_->getWayPointCount();
        printf("\nPath Size!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n", path_size);
        printf("%d\n\n", path_size);

        //DECLARATION for: update trajectory
        robot_state::JointState *jst;
        robot_state::RobotStatePtr curr_waypoint;
        const robot_model::JointModelGroup *group = res.trajectory_->getGroup();
        const std::vector<std::string> &active_joints = group->getJointModelNames();

        //DECLARATION for: sending small trajectory to controller
        TrajClient* traj_client_;
        traj_client_ = new TrajClient("/r_arm_controller/follow_joint_trajectory", true);

        while(!traj_client_->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the joint_trajectory_action server");
        }

        /*************************************************calculate and send trajectory online*********************************************************************/
        double push_coefficient = 0.3;
        double desire_joint_velocity[7];
        int waypoint_index = 0;

        double maximum_movement = 0;
        double second_maximum_movement = 0;
        int nearest_point_determine_index_1 = 0;
        int nearest_point_determine_index_2 = 0;
        //choose two joints to determine nearest waypoint for this path
        std::vector<double> start_positions;
        std::vector<double> goal_positions;
        res.trajectory_->getWayPoint(0).getJointStateGroup("right_arm")->getVariableValues(start_positions);
        res.trajectory_->getWayPoint(path_size - 1).getJointStateGroup("right_arm")->getVariableValues(goal_positions);
        //printf("11c\n");
        for (int joint_index = 0; joint_index < 7; joint_index++)
        {
            double temp_movement = std::abs(goal_positions[joint_index] - start_positions[joint_index]);
            if (joint_index != 4 && joint_index != 6)
            {
                if (temp_movement > maximum_movement)
                {
                    second_maximum_movement = maximum_movement;
                    nearest_point_determine_index_2 = nearest_point_determine_index_1;
                    maximum_movement = temp_movement;
                    nearest_point_determine_index_1 = joint_index;
                }
            }

            printf("temp_movement: %.3f", temp_movement);
            printf(" nearest_point_determine_index_1: %d", nearest_point_determine_index_1);
            printf(" nearest_point_determine_index_2: %d\n", nearest_point_determine_index_2);
        }

        bool sendOnce = true;
        while (waypoint_index < path_size - 1 )
        {
            //ros::Time begin = ros::Time::now();
            //ros::Duration time_duration;
            //printf("%d\n",waypoint_index);
            //printf("1\n");
            context_->planning_scene_monitor_->unlockSceneRead();
            ros::spinOnce();
            
            
            /******************************************************get current robot status (from controller publisher)****************************************************************/
            std::vector<double> temp_current_joint_position;
            //std::vector<double> temp_current_joint_velocity;

            for (int joint_index = 0; joint_index < 7; joint_index++)
            {
                temp_current_joint_position.push_back(current_joint_position[joint_index]);
                //temp_current_joint_velocity.push_back(current_joint_velocity[joint_index]);
            }
            
            robot_state::JointStateGroup* copied_joint_state_group = copied_state.getJointStateGroup("right_arm");
            copied_joint_state_group->setVariableValues(temp_current_joint_position);
            //printf("2\n");
            /******************************************************update locked planning scene****************************************************************/
            context_->planning_scene_monitor_->lockSceneRead ();
            const planning_scene::PlanningSceneConstPtr & updated_scene = context_->planning_scene_monitor_->getPlanningScene();
            //printf("3\n");
            
            /*****************************************************get nearest points************************************************************************/
            collision_request.group_name = "right_arm";
            collision_request.distance=true;
            collision_request.contacts = true;
            collision_request.verbose = true;
            collision_request.max_contacts = 1000;
            collision_result.clear();
            //printf("3\n");
            
            //planning scene monitor should be locked before any operation on the_scene
            updated_scene->getCollisionWorld()->checkRobotCollision(collision_request, collision_result, *updated_scene->getCollisionRobot(), copied_state);
            //the_scene->getCollisionWorld()->checkRobotCollision(collision_request, collision_result, *the_scene->getCollisionRobot(), copied_state);

            
            ros::param::get("/global_nearest_point1_x", collision_result.nearest_point_1(0));
            ros::param::get("/global_nearest_point1_y", collision_result.nearest_point_1(1));
            ros::param::get("/global_nearest_point1_z", collision_result.nearest_point_1(2));
            ros::param::get("/global_nearest_point2_x", collision_result.nearest_point_2(0));
            ros::param::get("/global_nearest_point2_y", collision_result.nearest_point_2(1));
            ros::param::get("/global_nearest_point2_z", collision_result.nearest_point_2(2));
            std::string nearest_point_link_name;
            ros::param::get("/nearest_point_link_name", nearest_point_link_name);

            //printf("4\n");
            
            /******************************************************jacobian matrix calculation****************************************************************/
            //rotation axis calculation (for finding jacobian reference point)
            const Eigen::Affine3d &joint_1_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_shoulder_pan_link")->getGlobalLinkTransform();
            const Eigen::Affine3d &joint_2_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_shoulder_lift_link")->getGlobalLinkTransform();
            const Eigen::Affine3d &joint_3_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_upper_arm_roll_link")->getGlobalLinkTransform();
            const Eigen::Affine3d &joint_4_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_elbow_flex_link")->getGlobalLinkTransform();
            const Eigen::Affine3d &joint_5_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_forearm_roll_link")->getGlobalLinkTransform();
            const Eigen::Affine3d &joint_6_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_wrist_flex_link")->getGlobalLinkTransform();
            const Eigen::Affine3d &joint_7_rotation_matrix = copied_joint_state_group->getRobotState()->getLinkState("r_wrist_roll_link")->getGlobalLinkTransform();
            Eigen::Vector3d nearest_point_joint_coordinate;
            Eigen::MatrixXd nearest_rotation_inverse_matrix = Eigen::MatrixXd::Random(3,3);
            //Eigen::MatrixXd nearest_rotation_matrix = Eigen::MatrixXd::Random(3,3);
            //printf("5\n");
            if (nearest_point_link_name == "r_shoulder_pan_link" || "head_tilt_link")
            {
                nearest_point_joint_coordinate=joint_1_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_1_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_1_rotation_matrix.rotation();
                nearest_point_link_name = "r_shoulder_pan_link";
            }
            else if (nearest_point_link_name == "r_shoulder_lift_link")
            {
                nearest_point_joint_coordinate=joint_2_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_2_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_2_rotation_matrix.rotation();
            }
            else if (	nearest_point_link_name == "r_upper_arm_roll_link" ||
                        nearest_point_link_name == "r_upper_arm_link")
            {
                nearest_point_joint_coordinate=joint_3_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_3_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_3_rotation_matrix.rotation();
                nearest_point_link_name="r_upper_arm_roll_link";
            }
            else if (nearest_point_link_name == "r_elbow_flex_link")
            {
                nearest_point_joint_coordinate=joint_4_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_4_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_4_rotation_matrix.rotation();
            }
            else if (	nearest_point_link_name == "r_forearm_roll_link" ||
                        nearest_point_link_name == "r_forearm_link")
            {
                nearest_point_joint_coordinate=joint_5_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_5_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_5_rotation_matrix.rotation();
                nearest_point_link_name="r_forearm_roll_link";
            }
            else if (nearest_point_link_name == "r_wrist_flex_link")
            {
                nearest_point_joint_coordinate=joint_6_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_6_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_6_rotation_matrix.rotation();
            }
            else if (	nearest_point_link_name == "r_wrist_roll_link"  ||
                        nearest_point_link_name =="r_gripper_motor_accelerometer_link"||
                        nearest_point_link_name =="r_gripper_motor_slider_link"||
                        nearest_point_link_name =="r_gripper_motor_screw_link"||
                        nearest_point_link_name == "l_gripper_palm_link" ||
                        nearest_point_link_name == "r_gripper_r_finger_link" ||
                        nearest_point_link_name == "r_gripper_l_finger_link" ||
                        nearest_point_link_name == "r_gripper_l_finger_tip_link" ||
                        nearest_point_link_name == "r_gripper_r_finger_tip_link" ||
                        nearest_point_link_name == "r_gripper_l_finger_tip_link")
            {
                nearest_point_joint_coordinate=joint_7_rotation_matrix.translation();
                nearest_rotation_inverse_matrix=joint_7_rotation_matrix.rotation().inverse();
                //nearest_rotation_matrix=joint_7_rotation_matrix.rotation();
                nearest_point_link_name="r_wrist_roll_link";
            }
            else
                std::cout << "nearest_point is not on right_arm group links !!!!!!!!!!!!!!!!!!!!!!!!  error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << std::endl;

            //printf("6\n");
            Eigen::Vector3d jacobian_reference_point;
            //?????????????????????????????????????do we want to multipy inverse rotation matrix or just rotation matrix, we have to check it latter
            jacobian_reference_point=nearest_rotation_inverse_matrix*(collision_result.nearest_point_2-nearest_point_joint_coordinate);
            //jacobian_reference_point=nearest_rotation_matrix*(collision_result.nearest_point_2-nearest_point_joint_coordinate);
            //jacobian_temp is 6x7, we only need first 3 rows
            Eigen::MatrixXd jacobian_temp;
            copied_joint_state_group->getJacobian(	nearest_point_link_name,
                                                    jacobian_reference_point,
                                                    jacobian_temp);
            //printf("7\n");
            Eigen::MatrixXd jacobian_matrix(3, 7);
            for (int i=0; i<3; i++)
            {
                for (int j=0; j<7; j++)
                {
                    jacobian_matrix(i,j) = jacobian_temp(i,j);
                }
            }
            //printf("8\n");
            
            /*************************************************show shortest distance vector********************************************************/

            //create a marker message
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom_combined";
            marker.header.stamp = ros::Time();
            marker.ns = "rviz_mesh_display";
            marker.id = waypoint_index + 1;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.01;
            marker.scale.y = 0.02;
            marker.scale.z = 0;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            geometry_msgs::Point onRobot;
            onRobot.x=collision_result.nearest_point_1(0);
            onRobot.y=collision_result.nearest_point_1(1);
            onRobot.z=collision_result.nearest_point_1(2);
            geometry_msgs::Point onObject;
            onObject.x=collision_result.nearest_point_2(0);
            onObject.y=collision_result.nearest_point_2(1);
            onObject.z=collision_result.nearest_point_2(2);
            marker.points.push_back(onRobot);
            marker.points.push_back(onObject);

            vis_pub.publish( marker );
            //printf("9\n");

            /******************************************************unit joint contribution calculation****************************************************************/
            Eigen::Vector3d distance_vector = collision_result.nearest_point_1 - collision_result.nearest_point_2;	//nearest point 2 is point on Robot, nearest point 1 is point on object

            double distance_vector_norm = std::sqrt(distance_vector(0)*distance_vector(0)+distance_vector(1)*distance_vector(1)+distance_vector(2)*distance_vector(2));
            Eigen::MatrixXd unit_joint_contribution(7,1);
            unit_joint_contribution = jacobian_matrix.transpose()*(distance_vector/distance_vector_norm);

            //printf("10\n");
            /******************************************************joint speed calculation****************************************************************/

            for (int joint_index = 0; joint_index < 7; joint_index++)
            {

                if (unit_joint_contribution(joint_index,0)*current_joint_velocity[joint_index] <= 0)
                {
                    //set to maximum velocity if not contribute to move nearest point to human
                    desire_joint_velocity[joint_index] = velocity_constraints[joint_index];
                }
                else
                {
                    //calculate speed accroding to contribution to move nearest point to human
                    desire_joint_velocity[joint_index] = std::sqrt(push_coefficient * distance_vector_norm/std::abs(unit_joint_contribution(joint_index,0)));
                    //if calculated speed is larger than speed limit, set the speed to speed limit
                    if (desire_joint_velocity[joint_index] > velocity_constraints[joint_index])
                    {
                        desire_joint_velocity[joint_index] = velocity_constraints[joint_index];
                    }
                    //update the sign of velocity (desire_joint_velocity will always be positive if not do so)
//                    if (current_joint_velocity[joint_index] >= 0)
//                        desire_joint_velocity[joint_index] = desire_joint_velocity[joint_index];
//                    else
//                        desire_joint_velocity[joint_index] = -desire_joint_velocity[joint_index];
                }
            }

            //printf("11\n");
            
            /******************************************************send modified trajectory to controller****************************************************************/
            control_msgs::FollowJointTrajectoryGoal goal;
            // First, the joint names, which apply to all waypoints
            goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
            goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
            goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
            goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
            goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
            goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
            goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
            //printf("11a\n");

            /************************Find next waypoint relative to current position**************************/
            //if waypoint_index == 0, next waypoint is 0.
            //printf("11b\n");
            if (waypoint_index != 0)
            {
                //printf("11d\n");
                //compare current position of two chosen joints to all waypoints of two chosen joints to find nearest_waypoint_index
                double minimum_difference = 1000;
                int nearest_waypoint_index;
                for (int waypoint_index_for_compare = waypoint_index; waypoint_index_for_compare < path_size; waypoint_index_for_compare++)
                {
                    std::vector<double> temp_positions;
                    double temp_2_joints_difference_sum;
                    res.trajectory_->getWayPoint(waypoint_index_for_compare).getJointStateGroup("right_arm")->getVariableValues(temp_positions);
                    temp_2_joints_difference_sum = std::abs(temp_positions[nearest_point_determine_index_1] - current_joint_position[nearest_point_determine_index_1]) + std::abs(temp_positions[nearest_point_determine_index_2] - current_joint_position[nearest_point_determine_index_2]);
                    //printf("index: %d", waypoint_index_for_compare);
                    //printf(" sum: %.3f\n", temp_2_joints_difference_sum);
                    //printf("temp_2_joints_difference_sum: %.3f  minimum_difference: %.3f  |  %.3f %.3f %.3f %.3f\n", temp_2_joints_difference_sum, minimum_difference, temp_positions[nearest_point_determine_index_1], current_joint_position[nearest_point_determine_index_1], temp_positions[nearest_point_determine_index_2], current_joint_position[nearest_point_determine_index_2]);
                    if (temp_2_joints_difference_sum < minimum_difference)
                    {
                        nearest_waypoint_index = waypoint_index_for_compare;
                        minimum_difference = temp_2_joints_difference_sum;
                    }
                }
                //printf("11e\n");
                //determine nearest_waypoint_index or nearest_waypoint_index + 1 as next waypoint
                std::vector<double> nearest_waypoint_positions;
                res.trajectory_->getWayPoint(nearest_waypoint_index).getJointStateGroup("right_arm")->getVariableValues(nearest_waypoint_positions);
                //printf("!!!!!!!!!!!!!!!!!!!!!1\n");
                //printf("nearest_waypoint_index: %d\n", nearest_waypoint_index);
                //printf("N-C: %.3f\n",nearest_waypoint_positions[nearest_point_determine_index_1] - current_joint_position[nearest_point_determine_index_1]);
                //printf("11f\n");
                
                if ((nearest_waypoint_positions[nearest_point_determine_index_1] - current_joint_position[nearest_point_determine_index_1]) * (goal_positions[nearest_point_determine_index_1]-start_positions[nearest_point_determine_index_1]) > 0)
                {
                    if (std::abs(nearest_waypoint_positions[nearest_point_determine_index_2] - current_joint_position[nearest_point_determine_index_2]) > 0.3 )
                    {
                        waypoint_index = nearest_waypoint_index;
                    }
                    else
                    {
                        waypoint_index = nearest_waypoint_index + 3;
                        if (waypoint_index > path_size - 1)
                        {
                            waypoint_index = path_size - 1;
                        }
                    }

                }
                else
                {
                    waypoint_index = nearest_waypoint_index + 3;
                    if (waypoint_index > path_size - 1)
                    {
                        waypoint_index = path_size - 1;
                    }

                }
            }
            else
            {
                waypoint_index = 3;
            }
            
            //printf("11\n");
            //printf("11g\n");
            
            /**********************set the speed for the rest of the path by setting positions and duration for each waypoint************/
            goal.trajectory.points.resize(path_size - waypoint_index);
            double time_from_start = 0;
            //printf("waypoint_index: %d\n", waypoint_index);
            //printf("12\n");
            double biggest_duration = 0;
            double temp_duration[7];
            double first_biggest_duration = 0;
            for (int rest_path_waypoint_index = waypoint_index; rest_path_waypoint_index < path_size; rest_path_waypoint_index++)
            {
                //Set Positions for each rest_path_waypoint
                std::vector<double> rest_path_waypoint_index_positions;
                res.trajectory_->getWayPoint(rest_path_waypoint_index).getJointStateGroup("right_arm")->getVariableValues(rest_path_waypoint_index_positions);
                int goal_trajectory_index = rest_path_waypoint_index - waypoint_index;
                //printf("13\n");
                for (int joint_index=0; joint_index<7; joint_index++)
                {
                    goal.trajectory.points[goal_trajectory_index].positions.resize(7);
                    goal.trajectory.points[goal_trajectory_index].positions[joint_index]=rest_path_waypoint_index_positions[joint_index];
                }
                //printf("14\n");
                //set duration for each rest_path_waypoint

                
                for (int joint_index = 0; joint_index < 7; joint_index++)
                {
                    double joint_difference_temp;
                    /**********calculate joint difference*********/
                    //if goal_trajectory_index = 0:
                    //calculate joint_difference_temp by minus current_joint_position
                    //else
                    //calculate joint_difference_temp by minus previous joint position
                    if (goal_trajectory_index == 0)
                    {
                        joint_difference_temp = std::abs(goal.trajectory.points[goal_trajectory_index].positions[joint_index] - current_joint_position[joint_index]);
                        //only need wrap when goal_trajectory_index == 0
                        if (joint_index == 4 || joint_index == 6)
                        {
                            double joint_difference_for_4_or_6 = std::abs(std::abs(current_joint_position[joint_index]) + std::abs(goal.trajectory.points[goal_trajectory_index].positions[joint_index]) - 6.283);
                            if (joint_difference_for_4_or_6 < joint_difference_temp)
                            {
                                joint_difference_temp = joint_difference_for_4_or_6;
                            }
                        }
                    }
                    else
                    {
                        joint_difference_temp = std::abs(goal.trajectory.points[goal_trajectory_index].positions[joint_index] - goal.trajectory.points[goal_trajectory_index - 1].positions[joint_index]);
                    }

                    temp_duration[joint_index] = std::abs(joint_difference_temp/desire_joint_velocity[joint_index]);
                }
                //printf("15\n");
                //set duration for each rest_path_waypoint
                //int biggest_duration_joint_index;
                for (int joint_index = 0; joint_index < 7; joint_index++)
                {
                    //printf("temp_duration[%d]: %.3f biggest_duration: %.3f \n", joint_index, temp_duration[joint_index], biggest_duration);
                    if (joint_index != 6)
		    {
                    	if (biggest_duration < temp_duration[joint_index])
                    	{
                        	biggest_duration = temp_duration[joint_index];
                        	//biggest_duration_joint_index = joint_index;
                    	}
		    }
                }
                //printf("%d\n", biggest_duration_joint_index);
                if (goal_trajectory_index == 1)
                {
                    first_biggest_duration = biggest_duration;
                }



                //almost stop the arm if distance_vector_norm is less than 8cm, improve this by found contact then stop
                //printf("16\n");
                if (collision_result.collision == true)
                {
                    biggest_duration = 100.0;
                    first_biggest_duration = 100.0;
                }
                //printf("17\n");
                time_from_start = time_from_start + biggest_duration;

                goal.trajectory.points[goal_trajectory_index].time_from_start = ros::Duration(time_from_start);
                /**********************debug template************************/
//                            printf("v[");
//                            for (int joint_index = 0; joint_index < 7; joint_index++)
//                            {
//                                printf("%.3f, ", desire_joint_velocity[joint_index]);
//                            }
//                            printf("]");
//                            printf(" duration[");
//                            for (int joint_index = 0; joint_index < 7; joint_index++)
//                            {
//                                printf("%.3f, ", temp_duration[joint_index]);
//                            }
//                            printf("]");
//                            printf(" duration: %.3f\n", biggest_duration);
                biggest_duration = 0;
            }
            
            printf("%.4f\n", first_biggest_duration);
            //printf("18\n");
            /***********************send trajectory to controller*****************************/
            //printf("current P: %.3f", current_joint_position[nearest_point_determine_index_1]);
            //printf("19\n");
            //printf(" first WP: %.3f \n", goal.trajectory.points[0].positions[nearest_point_determine_index_1]);
            //printf("20\n");

            traj_client_->sendGoal(goal);

            //printf("21\n");

            //            printf("         t[");
            //            for (int joint_index = 0; joint_index < 7; joint_index++)
            //            {
            //                printf("%.3f, ", temp_duration[joint_index]);
            //            }
            //            printf("]");
            //            printf("         d[");
            //            for (int joint_index = 0; joint_index < 7; joint_index++)
            //            {
            //                printf("%.3f, ", joint_difference[joint_index]);
            //            }
            //            printf("]\n");
            //time_duration = ros::Time::now() - begin;
            //printf("%.3f \n",time_duration.toSec());
            
        }
        printf("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!Done!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");


    }
    catch(std::runtime_error &ex)
    {
        ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
        ROS_ERROR("Planning pipeline threw an exception");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }

    convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
    action_res.error_code = res.error_code_;
}




bool move_group::MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan)
{    
    setMoveState(PLANNING);

    planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
    bool solved = false;
    planning_interface::MotionPlanResponse res;
    try
    {
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        robot_state::RobotState copied_state = plan.planning_scene_->getCurrentState();
        collision_request.group_name = "right_arm";
        collision_request.distance=true;
        collision_request.contacts = true;
        //collision_request.max_contacts = 1000;
        //collision_result.clear();
        plan.planning_scene_->getCollisionWorld()->checkRobotCollision(collision_request, collision_result, *plan.planning_scene_->getCollisionRobot(), copied_state);
        //the_scene->checkCollision(collision_request, collision_result);
        ROS_INFO_STREAM("\nTest 4: Current state is " << (collision_result.collision ? "in" : "not in") << " collision\n");
        printf("\n\ndistance between arm and cloeset object is %f \n\n", collision_result.distance);
        double temp_velocity_limit_factor = 1;
        if (collision_result.distance >= 1 || collision_result.distance <= 0)
        {
            temp_velocity_limit_factor = 1;
        }
        else
        {
            temp_velocity_limit_factor = collision_result.distance;
        }
        ros::param::set("/robot_description_planning/velocity_limit_factor", temp_velocity_limit_factor);


        solved = context_->planning_pipeline_->generatePlan(plan.planning_scene_, req, res);
    }
    catch(std::runtime_error &ex)
    {
        ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    catch(...)
    {
        ROS_ERROR("Planning pipeline threw an exception");
        res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    }
    if (res.trajectory_)
    {
        plan.plan_components_.resize(1);
        plan.plan_components_[0].trajectory_ = res.trajectory_;
        plan.plan_components_[0].description_ = "plan";
    }
    plan.error_code_ = res.error_code_;
    return solved;
}

void move_group::MoveGroupMoveAction::startMoveExecutionCallback() 
{
    setMoveState(MONITOR);
}

void move_group::MoveGroupMoveAction::startMoveLookCallback()
{
    setMoveState(LOOK);
}

void move_group::MoveGroupMoveAction::preemptMoveCallback()
{
    context_->plan_execution_->stop();
}

void move_group::MoveGroupMoveAction::setMoveState(MoveGroupState state)
{
    move_state_ = state;
    move_feedback_.state = stateToStr(state);
    move_action_server_->publishFeedback(move_feedback_);
}

#include <class_loader/class_loader.h> 
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)
