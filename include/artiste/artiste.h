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

#ifndef INCLUDE_ARTISTE_ARTISTE_H_
#define INCLUDE_ARTISTE_ARTISTE_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>

#include <rmi_driver/rmi_logger.h>

#include "artiste/image_analyzer.h"
#include "artiste/path_checker.h"
#include "artiste/path_creator.h"
#include "artiste/path_executor.h"

namespace artiste
{
class Artiste
{
public:
  Artiste(const ros::NodeHandle &nh);
  virtual ~Artiste();

  void start();

  void imageCb(const sensor_msgs::ImageConstPtr &image_msg);

  void startCartMoveCb(const std_msgs::Empty::ConstPtr &msg);

protected:
  ros::NodeHandle nh_;

  // Subs and pubs
  image_transport::Subscriber sub_image_;      // Image from a camera or image publisher
  image_transport::Publisher pub_path_image_;  // Modified image showing contours
  ros::Publisher pub_rmi_;                     // rmi command list
  ros::Publisher pub_path_;                    // nav_msg Path for display
  ros::Subscriber sub_start_move_;             // Will send the rmi command list

  image_transport::ImageTransport it_;

  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  // Local stuff
  ImageAnalyzer image_analyzer_;
  PathCreator path_creator_;
  PathExecutor path_executor_;
  PathChecker path_checker_;

  bool start_move_;

  std::string source_frame_;
  std::string target_frame_;

  rmi_driver::rmi_log::RmiLogger logger_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_ARTISTE_H_ */
