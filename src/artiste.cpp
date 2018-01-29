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
  pub_path_image_ = it_.advertise("image_out", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("/image_pub/path", 1);
  // start_move_sub_ = nh_.subscribe("start_cart_move", 1, &FrameDrawer::startCartMoveCb, this);

  pub_rmi_ = nh_.advertise<robot_movement_interface::CommandList>("command_list", 1);
}

Artiste::~Artiste()
{
}

void Artiste::start()
{
  std::string image_topic = nh_.resolveName("/image_pub/image_raw");
  sub_image_ = it_.subscribe(image_topic, 1, &Artiste::imageCb, this);
}

void Artiste::imageCb(const sensor_msgs::ImageConstPtr &image_msg)
{
  logger_.INFO() << "Image received";
}
} /* namespace artiste */
