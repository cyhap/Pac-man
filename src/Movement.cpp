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
  * @file Movement.cpp
  * @copyright 2019 Ethan Quist
  * @author Ethan Quist
  * @date 11/26/2019
  * @brief This Class provides functionality to control the turtlebot's movements.
  */

#include "Movement.hpp"

Movement::Movement() {
  linearVelocity = 0.0;
  angularVelocity = 0.0;
  objectSeen = false;
}

Movement::~Movement() {
}

void Movement::setLinearVelocity(float lv) {
  linearVelocity = lv;  // Setting the linear velocity to the input
  return;
}

void Movement::setAngularVelocity(float av) {
  angularVelocity = av;  // Setting the angular velocity to the input
  return;
}

void Movement::setObjectSeen(bool os) {
  objectSeen = os;  // Setting the object seen to the input
  return;
}

float Movement::getLinearVelocity() {
  return linearVelocity;  // Returning linear velocity
}

float Movement::getAngularVelocity() {
  return angularVelocity;  // Returning angular velocity
}

bool Movement::checkVisuals() {
  return objectSeen;  // Returning object seen
}
