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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv_bridge;

namespace artiste
{
ImageAnalyzer::ImageAnalyzer() : logger_("ImageAnalyzer", "/")
{
  background_image = cv::imread("/home/ros-industrial/rmi_ws/image.png", CV_LOAD_IMAGE_GRAYSCALE);
  ;
}

ImageAnalyzer::~ImageAnalyzer()
{
}

CvImagePtr ImageAnalyzer::newImageFromMsg(const sensor_msgs::ImageConstPtr& image_msg, bool color)
{
  CvImagePtr image;
  try
  {
    if (color)
    {
      image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      // image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC1);
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
  cv::Mat thresh;
  cv::Mat mask, bgmodel, fgmodel;

  cv::Mat I;
  // I = image->image;

  bool is_camera = false;

  cv::cvtColor(image->image, I, CV_GRAY2RGB);

  if ((image->image.size().width == 640) && image->image.size().height == 480)
  {
    roi_.x = 120;
    roi_.y = 200;
    roi_.width = 400;
    roi_.height = 270;

    is_camera = true;
  }
  else
  {
    roi_.x = 0;
    roi_.y = 0;
    roi_.width = image->image.size().width;
    roi_.height = image->image.size().height;
  }
  cv::Rect roi2 = roi_;

  cv::Mat region = image->image(roi2);

  // mask.create(I.size(), CV_8UC3);
  // mask = cv::Mat::zeros(I.size(), CV_8UC1);
  // cv::grabCut(I, mask, roi2, bgmodel, fgmodel, 1, cv::GC_INIT_WITH_RECT);

  // cv::Mat crop(image->image, cv::Rect(10, 10, image->image.size().width - 10, image->image.size().height - 10));

  // cv::threshold(image->image, thresh, 127, 255, cv::THRESH_BINARY_INV);
  // cv::threshold(crop, thresh, 127, 255, cv::THRESH_BINARY_INV);

  // cv::Mat sub = /*background_image -*/ image->image;
  // cv::Mat sub = mask;
  // cv::subtract(image->image, background_image, sub);

  // cv::Canny(image->image, thresh, 100, 200, 3);
  // cv::threshold(thresh, thresh, 127, 255, cv::THRESH_BINARY_INV);

  if (is_camera)
    cv::adaptiveThreshold(region, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 101, 20);
  else
    cv::threshold(image->image, thresh, 127, 255, cv::THRESH_BINARY_INV);

  // cv::Mat test(thresh, cv::Rect(0, 0, image->image.size().width + 10, image->image.size().height + 10));
  cv::findContours(thresh, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);

  if (is_camera)
  {
    for (auto&& contour : contours)
    {
      for (auto&& point : contour)
      {
        point.x += roi_.x;
        point.y += roi_.y;
      }
    }
  }

  return contours;
}

// void ImageAnalyzer::sortContours(ContourVec& contours, SortType type)
//{
//}

void ImageAnalyzer::sortContours(
    ContourVec& contours,
    std::function<bool(const std::vector<cv::Point>& left, const std::vector<cv::Point>& right)> sort_func)
{
  std::sort(contours.begin(), contours.end(), sort_func);
}

ContourVec ImageAnalyzer::approxContours(const ContourVec& contours, double epsilon)
{
  ContourVec contours_poly;
  contours_poly.resize(contours.size());

  for (int i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(contours[i], contours_poly[i], epsilon, true);
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

  cv::rectangle(image->image, roi_, cv::Scalar::all(0), 2, cv::LineTypes::LINE_AA);
}

} /* namespace artiste */
