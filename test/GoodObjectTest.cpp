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
  * @file GoodObjectTest.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 11/27/2019
  * @brief This TEST file is for testing the methods in the GoodObject derived class.
  */

#include "Object.hpp"
#include "GoodObject.hpp"
#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(GoodObject, construct_check) {

  Object::Pose locData;
  locData.x = 1.00;
  locData.y = 1.00;
  locData.z = 1.00;
  locData.roll = 1.00;
  locData.pitch = 1.00;
  locData.yaw = 1.00;

  GoodObject obj1(1,locData);

  ASSERT_EQ(obj1.checkCollect(),true);
}

TEST(GoodObject, objlocation) {

  Object::Pose locData;
  locData.x = 1.00;
  locData.y = 1.00;
  locData.z = 1.00;
  locData.roll = 1.00;
  locData.pitch = 1.00;
  locData.yaw = 1.00;

  GoodObject obj1(1,locData);
  Object::Pose objLoc = obj1.getLocation();

  EXPECT_EQ(objLoc.x,1.00);
  EXPECT_EQ(objLoc.y,1.00);
  EXPECT_EQ(objLoc.z,1.00);
  EXPECT_EQ(objLoc.roll,1.00);
  EXPECT_EQ(objLoc.pitch,1.00);
  EXPECT_EQ(objLoc.yaw,1.00);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "objects_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
