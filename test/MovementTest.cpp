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
  * @file MovementTest.cpp
  * @copyright 2019 Ethan Quist
  * @author Ethan Quist
  * @date 11/27/2019
  * @brief This TEST file is for testing the Movement methods.
  */

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "Movement.hpp"

TEST(Movement, collision) {
  // Collision distance currently set to 0.55
  Movement moveObj;

  float dist = 1.00;
  moveObj.updateMinDist(dist);
  ASSERT_TRUE(moveObj.getClearAhead());
  
  dist = 0.25;
  moveObj.updateMinDist(dist);
  ASSERT_FALSE(moveObj.getClearAhead());
}

TEST(Movement, velocities) {
  // Collision distance currently set to 0.55
  Movement moveObj;

  float dist1 = 1.00;
  moveObj.updateMinDist(dist1);
  std::pair<double, double> robotVels1 = moveObj.computeVelocities();
  std::pair<double, double> testVels1(0.5, 0.0);
  ASSERT_EQ(robotVels1,testVels1);
  
  float dist2 = 0.25;
  moveObj.updateMinDist(dist2);
  std::pair<double, double> robotVels2 = moveObj.computeVelocities();
  std::pair<double, double> testVels2(0.0, 1.0);
  ASSERT_EQ(robotVels2,testVels2);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "movement_test");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
