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
 * @file ImageProcessing.hpp
 * @copyright 2019 Corbyn Yhap
 * @author Corbyn Yhap
 * @date 11/26/2019
 * @brief This Class Provides functions to handle rgb and depth image sensor
 *  data.
 */

#pragma once

#include "Object.hpp"
#include <opencv2/core.hpp>

#include <memory>
#include <utility>

class ImageProcessing {
 public:
  ImageProcessing();
  ~ImageProcessing();
  std::shared_ptr<Object> identifyObject(const cv::Mat&);
  std::pair<const cv::Mat&, std::shared_ptr<Object> > applyMask(const cv::Mat&);
  std::pair<size_t, size_t> computeCentroid(const cv::Mat&);
  void computePose(const cv::Mat&, std::pair<size_t, size_t>,
                   std::shared_ptr<Object>);
};
