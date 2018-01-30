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

/**
 * \brief Create a nav_msgs::Path based on a ContourVec
 */
class PathCreator
{
public:
  PathCreator();
  virtual ~PathCreator();

  /**
   * \brief Set the size of the paper you are drawing on
   *
   * @param max_x Paper width
   * @param max_y Paper height
   */
  void init(double max_x, double max_y);

  /**
   * \brief Create a path from Contours created by ImageAnalyzer
   *
   * @param contours ContourVec created by ImageAnalyzer
   * @param tf Transform to use to convert the local ContourVec positions
   * @param image_height Height of the camera image
   * @param image_width Width of the camera image
   * @return The calculated and transformed Path
   */
  nav_msgs::Path createPath(const ContourVec &contours, const geometry_msgs::TransformStamped &tf, double image_height,
                            double image_width);

  /**
   * \brief Convert a cv::Point into a PoseStamped.
   *
   * @param pt the cv::Point to convert
   * @param frame_id Frame string to use in the Pose
   * @param x_scale Scale pt.x by this much
   * @param y_scale Scale pt.y by this much
   * @return A PoseStamped where the x and y have been scaled
   */
  geometry_msgs::PoseStamped pointToPoseScaled(const cv::Point &pt, const std::string &frame_id, double x_scale,
                                               double y_scale);

  /**
   * \brief Transform and add a Pose to the Path
   *
   * @param path [in,out] the Path that is being build
   * @param tf Transform to use
   * @param pose The pose to transform and add
   */
  void addPose(nav_msgs::Path &path, const geometry_msgs::TransformStamped &tf, const geometry_msgs::PoseStamped &pose);

protected:
  double max_x_;
  double max_y_;

  rmi_driver::rmi_log::RmiLogger logger_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_PATH_CREATOR_H_ */
