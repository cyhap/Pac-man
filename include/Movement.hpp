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
  * @file Movement.hpp
  * @copyright 2019 Ethan Quist
  * @author Ethan Quist
  * @date 11/26/2019
  * @brief This Class provides functionality to control the turtlebot's movements.
  */

#ifndef INCLUDE_MOVEMENT_HPP_
#define INCLUDE_MOVEMENT_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <gazebo_msgs/DeleteModel.h>
#include <stdlib.h>


class Movement {
 private:
  ros::NodeHandle n_;  // Node handler
  ros::Publisher pub_;  // Node Publisher
  ros::Subscriber sub_;  // Node Subscriber
  ros::ServiceClient client_;  // Node Service Client
  float linearVelocity;  ///< Turtlebot's linear velocity
  float angularVelocity;  ///< Turtlebot's angular velocity
  bool objectSeen;  ///< boolean flag for object detection


 public:
  /**
    *  @brief   This is the constructor for the Movement Class
    *  @param	  None
    *  @return	None
    */
    Movement();

    /**
    *  @brief   This is the destructor for the Movement Class
    *  @param	  None
    *  @return	None
    */
    virtual ~Movement();

    /**
    *  @brief   This function sets the linear velocity of the turtlebot
    *  @param	  lv linear velocity as float
    *  @return	None
    */
    void setLinearVelocity(float);

    /**
    *  @brief   This function sets the angular velocity of the turtlebot
    *  @param	  av angular velocity as float
    *  @return	None
    */
    void setAngularVelocity(float);

    /**
    *  @brief   This function sets the objectSeen member
    *  @param	  os boolean
    *  @return	None
    */
    void setObjectSeen(bool);

    /**
    *  @brief   This function retrieves the linear velocity
    *  @param	  None
    *  @return	float of linear velocity
    */
    float getLinearVelocity();

    /**
    *  @brief   This function retrieves the angular velocity
    *  @param	  None
    *  @return	float of angular velocity
    */
    float getAngularVelocity();

    /**
    *  @brief   This function checks the objectSeen member flag
    *  @param	  None
    *  @return	boolean for objectSeen
    */
    bool checkVisuals();
  /**
   *  @brief   This function controls the roaming robot
   *  @param   None
   *  @return  None
   */
  void roamCallBack(const kobuki_msgs::BumperEvent::ConstPtr&);

};

#endif  // INCLUDE_MOVEMENT_HPP_
