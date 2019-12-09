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

#include <stdlib.h>

#include <utility>

class Movement {
 private:
  // Turtlebot's current linear velocity
  double linearVelocity;
  // Turtlebot's current angular velocity
  double angularVelocity;
  // This is the boolean indicating whether the robot can move forward.
  bool clearAhead;
  // This is the minimum distance reading allowed before turning starts.
  float collisionDist;
  // This is linear velocity when path is clear
  double maxLinVel;
  // This is the angular velocity when path is not clear.
  double maxAngVel;

 public:
  /**
  *  @brief   This is the constructor for the Movement Class
  *  @param	  aColDist Minimum collision distance
  *  @param	  aLinVel Maximum linear velocity
  *  @param	  angVel Maximum angular velocity
  *  @return	None
  */
  Movement(const double &aColDist = 0.55, const double &aLinVel = 0.5,
           const double &angVel = 1);

  /**
  *  @brief   This is the destructor for the Movement Class
  *  @param	  None
  *  @return	None
  */
  virtual ~Movement();

  /**
  *  @brief    Uses the turtlebot sensor data to determine whether something is 
  *   in front of the robot. Sets the clearAhead variable accordingly.
  *  @param	  aDist float of minimum distance observed from laser scan
  *  @return	None
  */
  void updateMinDist(float);

  /**
  *  @brief   Checks the clearAhead boolean and updates the velocities
  *   accordingly. (Straight if clear ahead. Turning if obstacle in front.)
  *  @param	  None
  *  @return	pair of doubles for linear and angular velocities
  */
  std::pair<double, double> computeVelocities();

  /**
  *  @brief   Returns whether or not the robot can move forward. This function 
  *   was added so unit testing could take place on the updateMinDist function.
  *  @param	  None
  *  @return	boolean of clearAhead status
  */
  bool getClearAhead();
};

#endif  // INCLUDE_MOVEMENT_HPP_
