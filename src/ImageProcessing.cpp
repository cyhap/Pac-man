/*
 * Copyright (c) 2019, Ari Kupferberg, Ethan Quist, Corbyn Yhap
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

 /**
 * @file ImageProcessing.cpp
 * @copyright 2019 Corbyn Yhap
 * @author Corbyn Yhap
 * @date 11/27/2019
 * @brief This Class defines the functions that process RGB and Depth sensor
 *        data together.
 */


#include "ImageProcessing.hpp"

#include "GoodObject.hpp"

ImageProcessing::ImageProcessing() {
}

ImageProcessing::~ImageProcessing() {
}

std::shared_ptr<Object> ImageProcessing::identifyObject(
    const cv::Mat &aImage, const cv::Mat &aDepthImg) {
  // Fixme[Yhap] Fill in this function;
  (void) aImage;
  (void) aDepthImg;

  std::shared_ptr<Object> tReturn(new GoodObject(0, Object::Pose()));
  return tReturn;
}

std::pair<cv::Mat, std::shared_ptr<Object> > ImageProcessing::applyMask(
    const cv::Mat &aImage) {
  // Define the  BGR Masks used to detect blocks
  // Green Block
  cv::Scalar lowGreen(0, 200, 0);
  cv::Scalar highGreen(0, 255, 0);
  // Red Block
  cv::Scalar lowRed(0, 0, 200);
  cv::Scalar highRed(0, 0, 255);

  cv::Mat greenThresh, redThresh;

  cv::inRange(aImage, lowGreen, highGreen, greenThresh);
  cv::inRange(aImage, lowRed, highRed, redThresh);

  // Image Processing taking two different inputs since images updated separately
  // updating the object creating code portion.

  std::pair<cv::Mat, std::shared_ptr<Object> > tReturn;
  return tReturn;
}

std::pair<size_t, size_t> ImageProcessing::computeCentroid(
    const cv::Mat &aImage) {
  // Fixme[Yhap] Fill in this function;
  (void) aImage;
  std::pair<size_t, size_t> tReturn;
  return tReturn;
}
void ImageProcessing::computePose(const cv::Mat &aImage,
                                  std::pair<size_t, size_t> aPixelLoc,
                                  std::shared_ptr<Object> aObject) {
  // Fixme[Yhap] Fill in this function;
  (void) aImage;
  (void) aPixelLoc;
  (void) aObject;
}
