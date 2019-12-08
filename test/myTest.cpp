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
  * @file test.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 12/7/2019
  * @brief This TEST file is for level 2 integration testing.
  */

#include <string>

#include "ros/ros.h"
#include "gtest/gtest.h"

#include "pacman/ObjPose.h"  // Our custom msg data
#include "pacman/NavPose.h"  // Our custom srv type

/**
 * @brief Test Case ...
 * @param TESTSUITE group
 * @param testNavService description
 * @return None
 */
TEST(TESTSuite, testNavService) {
  // Create a Ros Node Handle
  ros::NodeHandle nTest;

  // Create a Service Client
  ros::ServiceClient client = nTest.serviceClient<pacman::NavPose>("navpose");

  // Check for Server
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);

  // Create a Pose
  pacman::ObjPose srvPose;
  pacman::NavPose srv;
  srv.request.pose = srvPose;

  // Send a Goal Pose to the Server
  client.call(srv);

  std::string expOutput = "Received Goal Pose";
  ASSERT_EQ(expOutput.compare(srv.response.str), 0);
}

/**
 * @brief Main function initializes ROS and runs all the TESTS created
 * @param	argc for ROS
 * @param	argv for ROS
 * @return Run all the tests
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "test_service");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
