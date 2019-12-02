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
  * @file Object.hpp
  * @copyright 2019 Ari Kupferberg
  * @author Ari Kupfeberg
  * @date 11/24/2019
  * @brief This Class is the Base Class for the Object Classes.
  */

#ifndef INCLUDE_OBJECT_HPP_
#define INCLUDE_OBJECT_HPP_

#include <vector>

class Object {
 public:
    struct Pose {
        double x = 0.00;
        double y = 0.00;
        double z = 0.00;
        double roll = 0.00;
        double pitch = 0.00;
        double yaw = 0.00;
    };

    /**
    *  @brief   This is the Constructor for the Object Class, with initializer list
    *  @param	  ind index of Object
    *  @param	  loc Pose location of Object
    *  @return	None
    */
    Object(int ind, Object::Pose loc) : index{ ind }, location{ loc } {}

    /**
    *  @brief   This is a function to get the Pose of the Object
    *  @param	  None
    *  @return	None
    */
    Pose getPose();

    /**
    *  @brief   This is a function to get the XYZ location of the Object
    *  @param	  None
    *  @return	None
    */
    std::vector<double> getXYZ();

 private:
    int index;  ///< index value of Object
    Pose location;  ///< 6-DOF location of Object as Pose
};

#endif  // INCLUDE_OBJECT_HPP_
