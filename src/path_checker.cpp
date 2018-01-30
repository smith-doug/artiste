/*
 * Copyright (c) 2018, Doug Smith, KEBA Corp
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *  Created on: Jan 30, 2018
 *      Author: Doug Smith
 */

#include <artiste/path_checker.h>

#include <moveit/move_group_interface/move_group_interface.h>

namespace artiste
{
PathChecker::PathChecker() : logger_("PathChecker", "/")
{
}

PathChecker::~PathChecker()
{
}

bool PathChecker::checkPath(const nav_msgs::Path &path, const std::string &manipulator)
{
  moveit::planning_interface::MoveGroupInterface move_group(manipulator);
  std::vector<geometry_msgs::Pose> waypoints;

  std::stringstream ss;

  // Keep a4 from flipping for no reason.
  moveit_msgs::Constraints path_constraints;
  path_constraints.name = "dont_flip";

  moveit_msgs::JointConstraint jc;
  jc.joint_name = "joint_4";
  jc.position = 0;
  jc.tolerance_above = M_PI_2;
  jc.tolerance_below = M_PI_2;
  jc.weight = 1;
  path_constraints.joint_constraints.push_back(jc);

  jc.joint_name = "joint_1";

  path_constraints.joint_constraints.push_back(jc);

  move_group.setPathConstraints(path_constraints);

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  auto current_pose = move_group.getCurrentPose();

  geometry_msgs::Pose arrow_pose;
  {
    std::vector<geometry_msgs::Pose> waypoints_to_start;
    auto start_target = path.poses.front();
    // start_target.pose.position.z += 0.01;

    arrow_pose = start_target.pose;
    arrow_pose.position.z += 0.1;
    // visual_tools_.publishZArrow(arrow_pose, rvt::WHITE, rvt::XSMALL, 0.1);
    // visual_tools_.publishSphere(start_target.pose, rvt::BLUE, rvt::XSMALL);

    // visual_tools_.trigger();

    move_group.setPoseTarget(start_target);
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if (move_group.plan(start_plan))
    {
      // move_group.setPathConstraints(path_constraints);
      move_group.move();
    }
    else
    {
      ROS_ERROR_STREAM("Failed to move to start pos");
      return false;
    }
  }

  move_group.setMaxVelocityScalingFactor(1);
  move_group.setMaxAccelerationScalingFactor(1);
  move_group.clearPathConstraints();

  ros::Duration(0.1).sleep();

  current_pose = move_group.getCurrentPose();

  // waypoints.push_back(this->current_pose_.pose);
  waypoints.push_back(current_pose.pose);

  for (auto &&pt : path.poses)
  {
    waypoints.push_back(pt.pose);
  }

  // move_group_arm.setMaxVelocityScalingFactor(1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0;
  const double eef_step = 0.01;

  auto text_pose = arrow_pose;
  ROS_INFO_STREAM("Attempting to compute cartesian path with moveit");
  double fraction = 0;
  for (int i = 0; i < 4 && fraction < 0.99; i++)
  {
    ROS_INFO_STREAM("Attempt #" << i << " of 4");
    ss.str("");
    ss << "Computenating cart path Attempt #" << i << " of 4";

    text_pose.position.x += 0.4;

    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, path_constraints);
  }

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);

  if (fraction > 0.99)
  {
    ROS_INFO_STREAM("MoveIt computeCartesianPath checked out ok with " << trajectory.joint_trajectory.points.size()
                                                                       << " points");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    // move_group_.asyncExecute(plan);
    // return false;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Path probably had a collision");
    return false;
  }
}

} /* namespace artiste */
