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

#ifndef INCLUDE_ARTISTE_PATH_EXECUTOR_H_
#define INCLUDE_ARTISTE_PATH_EXECUTOR_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <robot_movement_interface/CommandList.h>

#include <rmi_driver/rmi_logger.h>

namespace artiste
{
class PathExecutor
{
public:
  PathExecutor();
  virtual ~PathExecutor();

  robot_movement_interface::CommandList createCmdList(const nav_msgs::Path &path);

  virtual robot_movement_interface::Command poseToRmiCommand(const geometry_msgs::PoseStamped &pt);

  virtual std::vector<robot_movement_interface::Command>
  finalRmiCommands(const robot_movement_interface::CommandList &cmd_list, const nav_msgs::Path &path);

protected:
  rmi_driver::rmi_log::RmiLogger logger_;

  int cmd_id_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_PATH_EXECUTOR_H_ */
