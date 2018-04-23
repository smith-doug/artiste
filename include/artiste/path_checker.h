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

#ifndef INCLUDE_ARTISTE_PATH_CHECKER_H_
#define INCLUDE_ARTISTE_PATH_CHECKER_H_

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <nav_msgs/Path.h>

#include <rmi_driver/rmi_logger.h>

namespace artiste
{
using MovePlan = moveit::planning_interface::MoveGroupInterface::Plan;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

/**
 * \brief Checks the path using moveit's cartesian functionality.
 *
 * Note: It doesn't actually execute the path created by moveit computeCartesianPath.  If it manages to actually compute
 * the path based on waypoints from PathCreator, it's safe enough.
 */
class PathChecker
{
public:
  PathChecker();
  virtual ~PathChecker();

  bool checkPath(const nav_msgs::Path &path, const std::string &manipulator);

  bool planToStart(MoveGroup &move_group, const nav_msgs::Path &path, MovePlan &plan);

  bool moveToStart(MoveGroup &move_group, MovePlan &plan);

  bool computeCartesianPath(MoveGroup &move_group, const nav_msgs::Path &path, MovePlan *plan = 0,
                            moveit_msgs::Constraints *constraints = 0);

  std::vector<geometry_msgs::Pose> getWaypoints(MoveGroup &move_group, const nav_msgs::Path &path);

  virtual moveit_msgs::Constraints makeConstraints();

  void setPerformInitialMove(bool perform_initial_moves)
  {
    perform_initial_moves_ = perform_initial_moves;
  }

protected:
  bool perform_initial_moves_;
  rmi_driver::rmi_log::RmiLogger logger_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_PATH_CHECKER_H_ */
