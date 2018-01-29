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

#ifndef INCLUDE_ARTISTE_IMAGE_ANALYZER_H_
#define INCLUDE_ARTISTE_IMAGE_ANALYZER_H_

#include <boost/shared_ptr.hpp>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <rmi_driver/rmi_logger.h>

namespace artiste
{
using ContourVec = std::vector<std::vector<cv::Point>>;
class ImageAnalyzer
{
public:
  enum SortType
  {
    SortTypeSize,
    SortTypeLeft,
    SortTypeY,
    SortTypeBoth
  };

  ImageAnalyzer();
  virtual ~ImageAnalyzer();

  cv_bridge::CvImagePtr newImageFromMsg(const sensor_msgs::ImageConstPtr &image_msg, bool color = false);

  void flipImage(cv_bridge::CvImagePtr image);

  ContourVec findContours(cv_bridge::CvImagePtr image);

  void sortContours(ContourVec &contours, SortType type);

  void
  sortContours(ContourVec &contours,
               std::function<bool(const std::vector<cv::Point> &left, const std::vector<cv::Point> &right)> sort_func);

  ContourVec approxContours(const ContourVec &contours);

  void drawContours(cv_bridge::CvImagePtr, const ContourVec &contours);

protected:
  rmi_driver::rmi_log::RmiLogger logger_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_IMAGE_ANALYZER_H_ */
