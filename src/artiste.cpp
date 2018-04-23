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

#include <chrono>
#include <thread>

namespace artiste
{
Artiste::Artiste(const ros::NodeHandle &nh) : nh_(nh), logger_("artiste", "/"), it_(nh), tf_listener_(tf_buffer_)

{
  start_move_ = false;
  source_frame_ = "camera_frame";
  target_frame_ = "world";
  // path_creator_.init(0.18, 0.18);
  // path_creator_.init(0.12, 0.12);
  path_creator_.init(0.17, 0.17);

  pub_path_image_ = it_.advertise("image_out", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/image_pub/path", 1);

  pub_rmi_ = nh_.advertise<robot_movement_interface::CommandList>("command_list", 1);
}

Artiste::~Artiste()
{
}

void Artiste::start()
{
  path_checker_.setPerformInitialMove(true);  // Use moveit to move to the start point

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

  bool needs_update = false;

  if (!last_image_)
    needs_update = true;

  geometry_msgs::TransformStamped tf_camera;
  try
  {
    tf_camera = tf_buffer_.lookupTransform(target_frame_, source_frame_, ros::Time(0));
    if (first_time)
      logger_.INFO() << "Image received and transform ok";

    first_time = false;
  }
  catch (tf2::TransformException &ex)
  {
    if (first_time)
      logger_.INFO() << "Image received but failed to lookup transform.  Is it published? " << ex.what();
    else
      logger_.ERROR() << "Failed to lookup transform.  It worked before at least once. " << ex.what();

    return;
  }

  auto &camera_rot = tf_camera.transform.rotation;
  auto &last_rot = last_tf_.transform.rotation;

  auto &camera_tf = tf_camera.transform.translation;
  auto &last_tf = last_tf_.transform.translation;

  if (camera_rot.w != last_rot.w || camera_rot.x != last_rot.x || camera_rot.y != last_rot.y ||
      camera_rot.z != last_rot.z || camera_tf.x != last_tf.x || camera_tf.y != last_tf.y || camera_tf.z != last_tf.z)
  {
    logger_.INFO() << "Tf was new";
    needs_update = true;
  }

  last_tf_ = tf_camera;

  if (start_move_)
  {
    start_move_ = false;
    if (path_checker_.checkPath(path_, "pointer"))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      auto cmd_list = path_executor_.createCmdList(path_);
      pub_rmi_.publish(cmd_list);
    }
  }

  if (!needs_update && last_image_)
  {
    if (image_msg->data.size() == last_image_->data.size() && image_msg->data == last_image_->data)
    {
      return;
    }
  }

  last_image_ = image_msg;

  path_ = CreatePath(image_msg, tf_camera);

  logger_.INFO() << "New image received";
}

nav_msgs::Path Artiste::CreatePath(const sensor_msgs::ImageConstPtr &image_msg,
                                   geometry_msgs::TransformStamped &tf_camera)
{
  nav_msgs::Path path;
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

      // return (ra.y * 10 + ra.x) < (rb.y * 10 + rb.x);
      return (ra.x * ra.x + ra.y * ra.y) < (rb.x * rb.x + rb.y * rb.y);
    }
  };

  // Find, sort, and approximate the contours.
  auto contours = image_analyzer_.findContours(image);
  image_analyzer_.sortContours(contours, contour_sorter());
  auto contours_poly = image_analyzer_.approxContours(contours);

  // Draw the approx contours on the color image, flip it, the publish for rviz
  image_analyzer_.drawContours(image_color, contours_poly);
  image_analyzer_.flipImage(image_color);
  pub_path_image_.publish(image_color->toImageMsg());

  path = path_creator_.createPath(contours_poly, tf_camera, image_msg->height, image_msg->width);
  pub_path_.publish(path);

  return path;
}
} /* namespace artiste */
