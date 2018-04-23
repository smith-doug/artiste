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

/**
 * \brief Analyze an image for contours.
 */
class ImageAnalyzer
{
public:
  cv::Mat background_image;

  cv::Rect roi_;

  enum SortType
  {
    SortTypeSize,
    SortTypeLeft,
    SortTypeY,
    SortTypeBoth
  };

  ImageAnalyzer();
  virtual ~ImageAnalyzer();

  /**
   * \brief Create a CvImagePtr from a sensor message
   *
   * @param image_msg A sensor_msgs::Image from a camera or other publisher
   * @param color Make a color image or not
   * @return a new CvImagePtr
   */
  cv_bridge::CvImagePtr newImageFromMsg(const sensor_msgs::ImageConstPtr &image_msg, bool color = false);

  /**
   * \brief Flip the image around its y axis.
   *
   * @param [in,out] image The CvImage created by newImageFromMsg
   */
  void flipImage(cv_bridge::CvImagePtr image);

  /**
   * Find the contours and return them as a vector of vectors of points.
   *
   * @param image The cv image created by newImageFromMsg()
   * @return
   */
  ContourVec findContours(cv_bridge::CvImagePtr image);

  /**
   * \brief Sorts the contours
   *
   * @param contours [in,out] Contours created by findContours()
   * @param type
   */
  // virtual void sortContours(ContourVec &contours, SortType type);

  /**
   * \brief Sort the contours with a given sorting function
   *
   * @param contours [in,out] Contours created by findContours()
   * @param sort_func Sort function that can be passed to std::sort.
   */
  void
  sortContours(ContourVec &contours,
               std::function<bool(const std::vector<cv::Point> &left, const std::vector<cv::Point> &right)> sort_func);

  /**
   * \brief Approximate the contours to reduce some of the complexity with approxPolyDP
   *
   * @param contours Contours created by findContours()
   * @param epsilon value to use in approxPolyDP
   * @return a new ContourVec of the approximated contours
   */
  ContourVec approxContours(const ContourVec &contours, double epsilon = 1.0);

  /**
   * \brief Draw the contours on a CvImage with pretty colors
   *
   * @param image a CvImage to draw on
   * @param contours Contours created by findContours()
   */
  void drawContours(cv_bridge::CvImagePtr image, const ContourVec &contours);

protected:
  rmi_driver::rmi_log::RmiLogger logger_;
};

} /* namespace artiste */

#endif /* INCLUDE_ARTISTE_IMAGE_ANALYZER_H_ */
