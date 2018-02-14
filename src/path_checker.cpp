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

namespace artiste
{
template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp = 4)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::abs(x - y) <= std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp
         // unless the result is subnormal
         || std::abs(x - y) < std::numeric_limits<T>::min();
}

PathChecker::PathChecker() : logger_("PathChecker", "/"), perform_initial_moves_(false)
{
}

PathChecker::~PathChecker()
{
}

bool PathChecker::checkPath(const nav_msgs::Path &path, const std::string &manipulator)
{
  moveit::planning_interface::MoveGroupInterface move_group(manipulator);

  std::stringstream ss;

  // Keep a4 from flipping for no reason.
  moveit_msgs::Constraints path_constraints = makeConstraints();
  move_group.setPathConstraints(path_constraints);

  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  moveit::planning_interface::MoveGroupInterface::Plan start_plan;
  if (!planToStart(move_group, path, start_plan))
    return false;

  if (perform_initial_moves_ && !moveToStart(move_group, start_plan))
    return false;

  // move_group.clearPathConstraints();

  ros::Duration(0.5).sleep();

  // move_group.setStartStateToCurrentState();
  if (computeCartesianPath(move_group, path, 0))  // computeCartesianPath(move_group, path, 0, &path_constraints))
  {
    logger_.INFO() << "Cartesian path computed successfully, all's right with the world.";
    return true;
  }
  else
  {
    logger_.ERROR() << "moveit failed to create a cartesian plan based on the provided path.  Running is not safe.";
    return false;
  }
}

bool PathChecker::planToStart(MoveGroup &move_group, const nav_msgs::Path &path, MovePlan &plan)
{
  auto start_target = path.poses.front();
  // start_target.pose.position.z += 0.01;

  move_group.setPoseTarget(start_target);

  if (move_group.plan(plan))
  {
    logger_.INFO() << "Planned move to start position OK.";
    return true;
  }
  else
  {
    logger_.ERROR() << "Failed to move to start pos";
    return false;
  }
}

bool PathChecker::moveToStart(MoveGroup &move_group, MovePlan &plan)
{
  logger_.INFO() << "Moving to start position";
  if (move_group.execute(plan))
  {
    logger_.INFO() << "Moved to start OK";
    return true;
  }
  else
  {
    logger_.ERROR() << "Failed to move to start position";
    return false;
  }
}

bool PathChecker::computeCartesianPath(MoveGroup &move_group, const nav_msgs::Path &path, MovePlan *plan,
                                       moveit_msgs::Constraints *constraints)
{
  // Change the nav_msgs::Path into a waypoint vector, starting with the current pose
  std::vector<geometry_msgs::Pose> waypoints;

  waypoints.push_back(move_group.getCurrentPose().pose);
  for (auto &&pt : path.poses)
  {
    waypoints.push_back(pt.pose);
  }

  // move_group_arm.setMaxVelocityScalingFactor(1);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0;
  const double eef_step = 0.05;

  // move_group.setPlanningTime(15);
  // move_group.allowReplanning(true);
  logger_.INFO() << "Attempting to compute cartesian path with moveit";

  double fraction = 0;
  moveit_msgs::MoveItErrorCodes ec;
  for (int i = 0; !almost_equal(fraction, 1.00) && i < 5; i++)  // Try a couple times
  {
    logger_.INFO() << "Attempt #" << i + 1 << " of 5";

    if (constraints)
    {
      fraction =
          move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, *constraints, true, &ec);
    }
    else
    {
      fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, true, &ec);
    }
    logger_.INFO() << (fraction * 100.0) << "% planned";
  }

  logger_.INFO() << "Cartesian path (" << fraction * 100.0 << "%) achieved";

  if (almost_equal(fraction, 1.00))
  {
    logger_.INFO() << "MoveIt computeCartesianPath checked out ok with " << trajectory.joint_trajectory.points.size()
                   << " points";

    if (plan)
      plan->trajectory_ = trajectory;

    // move_group_.asyncExecute(plan);
    // return false;
    return true;
  }
  else
  {
    logger_.ERROR() << "Path probably had a collision.  Ec was: " << ec.val;
    return false;
  }
}

moveit_msgs::Constraints PathChecker::makeConstraints()
{
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

  jc.joint_name = "joint_6";
  jc.tolerance_above = M_PI;
  jc.tolerance_below = M_PI;
  path_constraints.joint_constraints.push_back(jc);

  return path_constraints;
}

} /* namespace artiste */
