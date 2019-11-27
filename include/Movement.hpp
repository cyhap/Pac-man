/*
 * @copyright Copyright 2019 <Ethan Quist>
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
 *
 * @author Ethan Quist
 *
 * @file Movement.hpp
 *
 * @brief This is header filef for the movmenet class to control the turtlebot.
 *
 */

#pragma once

class Movement {
 public:

  /**
   * @brief constructor for Movment Class
   *
   * @param none
   * @return none
   */
  Movement();

  /**
   * @brief constructor for Movment Class
   *
   * @param none
   * @return none
   */
  virtual ~Movement();

  /**
   * @brief Sets the linear velocity of the turtlebot
   *
   * @param float for linear velocity
   * @return none
   *
   */
  void setLinearVelocity(float);

  /**
   * @brief Sets the linear velocity of the turtlebot
   *
   * @param float for linear velocity
   * @return none
   *
   */
  void setAngularVelocity(float);

  /**
   * @brief Sets the linear velocity of the turtlebot
   *
   * @param float for linear velocity
   * @return none
   *
   */
  void setObjectSeen(bool);

  /**
   * @brief retrieve the linear velocity
   *
   * @param none
   * @return float of the linear velocity
   */
  float getLinearVelocity();

  /**
   * @brief retrieve the angular velocity
   *
   * @param none
   * @return float of the angular velocity
   */
  float getAngularVelocity();

  /**
   * @brief checks the seen object flag
   *
   * @param none
   * @return bool if objectSeen is true
   */
  bool checkVisuals();

 private:
  // This is the value for turtlebot's linear velocity
  float linearVelocity;
  // This is the value for the turtlebot's angular velocity
  float angularVelocity;
  // This flags if an object has been detected or not
  bool objectSeen;

};



