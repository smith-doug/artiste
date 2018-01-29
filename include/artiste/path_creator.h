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

#ifndef INCLUDE_ARTISTE_PATH_CREATOR_H_
#define INCLUDE_ARTISTE_PATH_CREATOR_H_

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <robot_movement_interface/CommandList.h>

#include <rmi_driver/rmi_logger.h>

namespace artiste
{
using ContourVec = std::vector<std::vector<cv::Point>>;
class PathCreator
{
public:
  PathCreator();
  virtual ~PathCreator();

  void init(double max_x, double max_y, std::string source_frame, std::string dest_frame);

  nav_msgs::Path createPath(const ContourVec &contours, const geometry_msgs::TransformStamped &tf, double image_height,
                            double image_width);

  geometry_msgs::PoseStamped pointToPoseScaled(const cv::Point &pt, double x_scale, double y_scale);

  void addPose(nav_msgs::Path &path, const geometry_msgs::TransformStamped &tf, const geometry_msgs::PoseStamped &pose);

protected:
  double max_x_;
  double max_y_;
  std::string source_frame_;
  std::string dest_frame_;

  rmi_driver::rmi_log::RmiLogger logger_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_PATH_CREATOR_H_ */
