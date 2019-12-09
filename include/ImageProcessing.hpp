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

#ifndef INCLUDE_IMAGEPROCESSING_HPP_
#define INCLUDE_IMAGEPROCESSING_HPP_


#include <memory>
#include <vector>

#include "Object.hpp"
#include "opencv2/core.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class ImageProcessing {
 public:
  /**
   *  @brief  Image Processing Constructor. Initializes with Green Mask for good
   *          objects and a Red Mask for bad objects.
   *  @param  None
   *  @return None
   */
  ImageProcessing();

  /**
   *  @brief  Image Processing Destructor
   *  @param  None
   *  @return None
   */
  ~ImageProcessing();

  /**
   *  @brief  process Use this to perform the image processing and pose
   *          extraction.
   *  @param  None.
   *  @return std::vector<std::shared_ptr<Object> > A vector of all objects
   *          (good and bad) found within the image.
   */
  std::vector<std::shared_ptr<Object> > process();

  /**
   *  @brief  setRgbImg Set the RGB Image Used for Processing
   *  @param  std::shared_ptr<const cv::Mat> Pointer to the Image that should be
   *          processed.
   *  @return bool Whether or not the image submitted was accepted for
   *          processing
   */
  bool setRgbImg(std::shared_ptr<const cv::Mat>);

  /**
   *  @brief  setPntCld Set the Point Cloud Used for Processing
   *  @param  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >
   *          Pointer to the Point cloud that should be processed.
   *  @return bool Whether or not the point cloud submitted was accepted for
   *          processing
   */
  bool setPntCld(std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >);

  /**
   *  @brief  setGoodObjectMask Update the Mask Used to Identify Good Objects
   *  @param  const cv::Scalar& Lower Threshold of the Mask in BGR
   *  @param  const cv::Scalar& Upper Threshold of the Mask in BGR
   *  @return None.
   */
  void setGoodObjectMask(const cv::Scalar&, const cv::Scalar&);

  /**
   *  @brief  setBadObjectMask Update the Mask Used to Identify Bad Objects
   *  @param  const cv::Scalar& Lower Threshold of the Mask in BGR
   *  @param  const cv::Scalar& Upper Threshold of the Mask in BGR
   *  @return None.
   */
  void setBadObjectMask(const cv::Scalar&, const cv::Scalar&);

  /**
   *  @brief  setPixelForPose Choose whether the Poses returned in objects
   *          are pixel locations or computed poses
   *  @param  bool true indicating you want the pixel location
   *  @return None.
   */
  void setPixelForPose(bool);

  /**
   *  @brief  applyGoodMask Apply the thresholds corresponding to good objects.
   *  @param  const cv::Mat& Image to Mask
   *  @return cv::mat The Masked Image
   */
  cv::Mat applyGoodMask(const cv::Mat&);

  /**
   *  @brief  applyBadMask Apply the thresholds corresponding to bad objects.
   *  @param  const cv::Mat& Image to Mask
   *  @return cv::mat The Masked Image
   */
  cv::Mat applyBadMask(const cv::Mat&);

 private:
  std::shared_ptr<const cv::Mat> rgbImg;
  std::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> > rectPntCld;
  cv::Scalar lowGood;
  cv::Scalar lowBad;
  cv::Scalar highGood;
  cv::Scalar highBad;
  bool pixelForPose;


  /**
   *  @brief  processMask helper function to identify object centers in the
   *          masked image.
   *  @param  const cv::Mat & The Masked Image.
   *  @return std::vector<Object::Pose> A vector of poses corresponding to
   *          object centers.
   */
  std::vector<Object::Pose> processMask(const cv::Mat&);

  /**
   *  @brief  extractPose Use the pixel index to extract a Pose from a point
   *          cloud
   *  @param  int x position in image
   *  @param  int y position in image
   *  @return Object::Pose Pose with corresponding XYZ positions from the point
   *          cloud.
   */
  Object::Pose extractPose(int, int);
};

#endif  // INCLUDE_IMAGEPROCESSING_HPP_
