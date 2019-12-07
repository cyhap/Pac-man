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
  * @file ObjectTest.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 11/27/2019
  * @brief This TEST file is for testing the methods in the GoodObject derived class.
  */

#include "Object.hpp"
#include "GoodObject.hpp"
#include "BadObject.hpp"

#include "ros/ros.h"
#include "gtest/gtest.h"

TEST(GoodObject, collecttrue) {
  Object::Pose locData;
  GoodObject obj1(locData);

  ASSERT_TRUE(obj1.checkCollect());
}

TEST(GoodObject, goodobjxyz) {
  Object::Pose locData;
  locData.x = 1.00;
  locData.y = 2.00;
  locData.z = 3.00;

  GoodObject obj1(locData);
  std::vector<double> objXYZ = obj1.getXYZ();
  std::vector<double> testXYZ{1.00, 2.00, 3.00};

  ASSERT_EQ(objXYZ, testXYZ);
}

TEST(GoodObject, goodobjpose) {
  Object::Pose locData;
  locData.x = 1.00;
  locData.y = 4.00;
  locData.z = 3.00;
  locData.roll = 2.00;
  locData.pitch = 1.50;
  locData.yaw = 0.77;

  GoodObject obj1(locData);
  Object::Pose objLoc = obj1.getPose();

  EXPECT_EQ(objLoc.x, locData.x);
  EXPECT_EQ(objLoc.y, locData.y);
  EXPECT_EQ(objLoc.z, locData.z);
  EXPECT_EQ(objLoc.roll, locData.roll);
  EXPECT_EQ(objLoc.pitch, locData.pitch);
  EXPECT_EQ(objLoc.yaw, locData.yaw);
}

TEST(BadObject, collectfalse) {
  Object::Pose locData;
  BadObject obj1(locData);

  ASSERT_FALSE(obj1.checkCollect());
}

TEST(BadObject, badobjxyz) {
  Object::Pose locData;
  locData.x = 7.00;
  locData.y = 0.00;
  locData.z = 2.00;

  BadObject obj1(locData);
  std::vector<double> objXYZ = obj1.getXYZ();
  std::vector<double> testXYZ{7.00, 0.00, 2.00};

  ASSERT_EQ(objXYZ, testXYZ);
}

TEST(BadObject, badobjpose) {
  Object::Pose locData;
  locData.x = 1.00;
  locData.y = 2.00;
  locData.z = 3.00;
  locData.roll = 4.00;
  locData.pitch = 5.00;
  locData.yaw = 6.00;

  BadObject obj1(locData);
  Object::Pose objLoc = obj1.getPose();

  EXPECT_EQ(objLoc.x, locData.x);
  EXPECT_EQ(objLoc.y, locData.y);
  EXPECT_EQ(objLoc.z, locData.z);
  EXPECT_EQ(objLoc.roll, locData.roll);
  EXPECT_EQ(objLoc.pitch, locData.pitch);
  EXPECT_EQ(objLoc.yaw, locData.yaw);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "objects_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
