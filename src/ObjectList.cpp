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
  * @file ObjectList.cpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 11/24/2019
  * @brief This Class is for storing and listing the collected objects
  */

#include "ObjectList.hpp"
#include "Object.hpp"
//#include "GoodObject.hpp"
//#include <memory>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

ObjectList::ObjectList() {
  numberOfObjects = 0;
}

ObjectList::~ObjectList() {}

int ObjectList::addObjectFound(Object::Pose objpose) {
//  int objNumber = numberOfObjects+1;
//  std::shared_ptr<Object> obj(new GoodObject(objNumber, objpose));

  objectsFound.emplace_back(objpose);
  numberOfObjects = objectsFound.size();
  return numberOfObjects;
}

void ObjectList::objsCallback(const geometry_msgs::Point::ConstPtr& msg) {
  Object::Pose pose;
  pose.x = msg->x;
  pose.y = msg->y;
  pose.z = msg->z;
  int count_ = addObjectFound(pose);
  ROS_INFO_STREAM("An Object Pose has been added to list of collected objects!");
  ROS_INFO_STREAM("There are " << count_ << " objects collected");
  if (count_ == 5) {
    ROS_WARN_STREAM("Collected All Objects!");
    for (auto element : objectsFound)
      ROS_INFO_STREAM("[" << element.x << "," << element.y << "," << element.z << "]");
  }
}

