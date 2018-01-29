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
#include <ros/ros.h>

#include <artiste/image_analyzer.h>

using namespace cv_bridge;

namespace artiste
{
ImageAnalyzer::ImageAnalyzer() : logger_("ImageAnalyzer", "/")
{
  // TODO Auto-generated constructor stub
}

ImageAnalyzer::~ImageAnalyzer()
{
  // TODO Auto-generated destructor stub
}

CvImagePtr ImageAnalyzer::newImageFromMsg(const sensor_msgs::ImageConstPtr& image_msg, bool color)
{
  CvImagePtr image;
  try
  {
    if (color)
    {
      image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    else
    {
      image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    }
  }
  catch (cv_bridge::Exception& ex)
  {
    logger_.ERROR() << "Failed to convert image";
    return nullptr;
  }

  return image;
}

void ImageAnalyzer::flipImage(cv_bridge::CvImagePtr image)
{
  cv::flip(image->image, image->image, 1);
}

ContourVec ImageAnalyzer::findContours(cv_bridge::CvImagePtr image)
{
  ContourVec contours;
  int max_thresh = 255;
  cv::Mat thresh;

  cv::threshold(image->image, thresh, 127, 255, cv::THRESH_BINARY_INV);
  cv::findContours(thresh, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);

  return contours;
}

void ImageAnalyzer::sortContours(ContourVec& contours, SortType type)
{
}

void ImageAnalyzer::sortContours(
    ContourVec& contours,
    std::function<bool(const std::vector<cv::Point>& left, const std::vector<cv::Point>& right)> sort_func)
{
  std::sort(contours.begin(), contours.end(), sort_func);
}

ContourVec ImageAnalyzer::approxContours(const ContourVec& contours)
{
  ContourVec contours_poly;
  contours_poly.resize(contours.size());

  for (int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(contours[i], contours_poly[i], 1, true);
  }

  return contours_poly;
}

void ImageAnalyzer::drawContours(cv_bridge::CvImagePtr image, const ContourVec& contours)
{
  cv::RNG rng(12345);

  for (int i = 0; i < contours.size(); ++i)
  {
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(image->image, contours, i, color, 2, cv::LineTypes::LINE_AA);
  }
}

} /* namespace artiste */
