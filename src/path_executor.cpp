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
 *  Created on: Jan 29, 2018
 *      Author: Doug Smith
 */

#include <artiste/path_executor.h>

namespace artiste
{
PathExecutor::PathExecutor() : logger_("PathExecutor", "")
{
  // TODO Auto-generated constructor stub
}

PathExecutor::~PathExecutor()
{
  // TODO Auto-generated destructor stub
}

robot_movement_interface::CommandList PathExecutor::createCmdList(const nav_msgs::Path &path)
{
  robot_movement_interface::CommandList cmd_list;
  for (auto &&pt : path.poses)
  {
    cmd_list.commands.emplace_back(poseToRmiCommand(pt));
  }

  // Use PTP for the 1st move.
  // Not needed now, path checker will move to the start position
  // cmd_list.commands[0].command_type = "PTP";

  logger_.INFO() << "CommandList has " << cmd_list.commands.size() << " entries";
  return cmd_list;
}

robot_movement_interface::Command PathExecutor::poseToRmiCommand(const geometry_msgs::PoseStamped &pt)
{
  robot_movement_interface::Command cmd;
  cmd.header.frame_id = "base_link";
  cmd.command_type = "LIN";
  cmd.pose_type = "QUATERNION";
  cmd.pose.push_back(pt.pose.position.x);
  cmd.pose.push_back(pt.pose.position.y);
  cmd.pose.push_back(pt.pose.position.z);
  cmd.pose.push_back(pt.pose.orientation.w);
  cmd.pose.push_back(pt.pose.orientation.x);
  cmd.pose.push_back(pt.pose.orientation.y);
  cmd.pose.push_back(pt.pose.orientation.z);

  cmd.velocity_type = "%";
  cmd.velocity.push_back(100);
  cmd.blending.push_back(1);

  return cmd;
}

} /* namespace artiste */
