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

#include <artiste/path_creator.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace artiste
{
PathCreator::PathCreator() : max_x_(0.18), max_y_(0.18), logger_("PathCreator", "")
{
}

void PathCreator::init(double max_x, double max_y)
{
  max_x_ = max_x;
  max_y_ = max_y;
}

PathCreator::~PathCreator()
{
}

nav_msgs::Path PathCreator::createPath(const ContourVec &contours, const geometry_msgs::TransformStamped &tf,
                                       double image_height, double image_width)
{
  nav_msgs::Path path;

  path.header.frame_id = tf.header.frame_id;

  auto child_frame = tf.child_frame_id;

  double x_scale = max_x_ / image_width;
  double y_scale = max_y_ / image_height;

  if (x_scale > y_scale)
    x_scale = y_scale;
  else
    y_scale = x_scale;

  geometry_msgs::PoseStamped temp_pose;
  for (auto &&cont : contours)
  {
    auto start_point = cont.front();
    auto end_point = cont.back();

    temp_pose = pointToPoseScaled(start_point, child_frame, x_scale, y_scale);

    auto start_pose = temp_pose;  // Save a copy to go back at end
    temp_pose.pose.position.z = 0.01;
    addPose(path, tf, temp_pose);

    for (auto &pt : cont)
    {
      temp_pose = pointToPoseScaled(pt, child_frame, x_scale, y_scale);
      addPose(path, tf, temp_pose);
    }

    addPose(path, tf, start_pose);

    temp_pose = start_pose;
    temp_pose.pose.position.z = 0.01;
    addPose(path, tf, temp_pose);
  }

  temp_pose.pose.position.z += 0.1;
  addPose(path, tf, temp_pose);

  return path;
}

geometry_msgs::PoseStamped PathCreator::pointToPoseScaled(const cv::Point &pt, const std::string &frame_id,
                                                          double x_scale, double y_scale)
{
  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header.frame_id = frame_id;
  temp_pose.pose.position.x = pt.x * x_scale;
  temp_pose.pose.position.y = pt.y * y_scale;
  temp_pose.pose.position.z = 0.0;
  temp_pose.pose.orientation.x = 0;
  temp_pose.pose.orientation.y = 1;
  temp_pose.pose.orientation.z = 0;
  temp_pose.pose.orientation.w = 0;

  return temp_pose;
}

void PathCreator::addPose(nav_msgs::Path &path, const geometry_msgs::TransformStamped &tf,
                          const geometry_msgs::PoseStamped &pose)
{
  geometry_msgs::PoseStamped pose_transformed;
  tf2::doTransform(pose, pose_transformed, tf);
  path.poses.push_back(pose_transformed);
}

} /* namespace artiste */
