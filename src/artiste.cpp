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

#include "artiste/artiste.h"

#include <nav_msgs/Path.h>
#include <robot_movement_interface/CommandList.h>

namespace artiste
{
Artiste::Artiste(const ros::NodeHandle &nh) : nh_(nh), logger_("artiste", "/"), it_(nh), tf_listener_(tf_buffer_)

{
  start_move_ = false;
  source_frame_ = "camera_frame";
  target_frame_ = "world";
  path_creator_.init(0.18, 0.18);

  pub_path_image_ = it_.advertise("image_out", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/image_pub/path", 1);

  pub_rmi_ = nh_.advertise<robot_movement_interface::CommandList>("command_list", 1);
}

Artiste::~Artiste()
{
}

void Artiste::start()
{
  ros::Duration(1.5).sleep();
  std::string image_topic = nh_.resolveName("/image_pub/image_raw");
  sub_image_ = it_.subscribe(image_topic, 1, &Artiste::imageCb, this);
  sub_start_move_ = nh_.subscribe("start_cart_move", 1, &Artiste::startCartMoveCb, this);
}

void Artiste::startCartMoveCb(const std_msgs::Empty::ConstPtr &msg)
{
  start_move_ = true;
  logger_.INFO() << "Starting a cart move";
}

void Artiste::imageCb(const sensor_msgs::ImageConstPtr &image_msg)
{
  static bool first_time = true;
  if (first_time)
  {
    logger_.INFO() << "Image received";
    first_time = false;
  }

  // Create and flip the images so they are the right way up after making the paths.  I could probably just use the
  // transform to rotate them but this works well.
  auto image = this->image_analyzer_.newImageFromMsg(image_msg, false);
  image_analyzer_.flipImage(image);

  auto image_color = this->image_analyzer_.newImageFromMsg(image_msg, true);
  image_analyzer_.flipImage(image_color);

  struct contour_sorter  // 'less' for contours
  {
    bool operator()(const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
    {
      auto &ra = a[0];
      auto &rb = b[0];
      // scale factor for y should be larger than img.width
      // return ((ra.y + 1.5 * ra.x) < (rb.y + 1.5 * rb.x));
      return (ra.y * 10 + ra.x) < (rb.y * 10 + rb.x);
    }
  };

  auto contours = image_analyzer_.findContours(image);
  image_analyzer_.sortContours(contours, contour_sorter());

  auto contours_poly = image_analyzer_.approxContours(contours);
  image_analyzer_.drawContours(image_color, contours_poly);

  image_analyzer_.flipImage(image_color);
  pub_path_image_.publish(image_color->toImageMsg());

  auto tf_camera = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));

  auto path = path_creator_.createPath(contours_poly, tf_camera, image_msg->height, image_msg->width);
  pub_path_.publish(path);

  if (start_move_)
  {
    start_move_ = false;
    auto cmd_list = path_executor_.createCmdList(path);
    pub_rmi_.publish(cmd_list);
  }
}
} /* namespace artiste */
