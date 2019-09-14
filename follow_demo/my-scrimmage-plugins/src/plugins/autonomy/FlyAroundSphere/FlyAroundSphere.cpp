/*!
* @file
*
* @section LICENSE
*
* Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
*
* This file is part of SCRIMMAGE.
*
*   SCRIMMAGE is free software: you can redistribute it and/or modify it under
*   the terms of the GNU Lesser General Public License as published by the
*   Free Software Foundation, either version 3 of the License, or (at your
*   option) any later version.
*
*   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
*   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
*   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
*   License for more details.
*
*   You should have received a copy of the GNU Lesser General Public License
*   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
*
* @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
* @author Eric Squires <eric.squires@gtri.gatech.edu>
* @date 31 July 2017
* @version 0.1.0
* @brief Brief file description.
* @section DESCRIPTION
* A Long description goes here.
*
*/

#include <my-scrimmage-plugins/plugins/autonomy/FlyAroundSphere/FlyAroundSphere.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>

#include <scrimmage/common/Shape.h>
#include <scrimmage/math/State.h>
#include <scrimmage/math/Angles.h>
#include <scrimmage/parse/ParseUtils.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;
#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h> // scrimmage::set()
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::FlyAroundSphere,
  FlyAroundSphere_plugin)

  namespace scrimmage {
    namespace autonomy {

      void FlyAroundSphere::init(std::map<std::string, std::string> &params) {
        initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);


        auto circle = sc::shape::make_circle(Eigen::Vector3d(0,0,50), sc::Quaternion(0,0,0), 50,
        Eigen::Vector3d(0, 0, 255), 0.25);
        // draw_shape(circle);
      }

      bool FlyAroundSphere::step_autonomy(double t, double dt) {
        Eigen::Vector3d &pos = state_->pos();
        Eigen::Vector3d &vel = state_->vel();
        double radius = 20;
        double w = initial_speed_/radius;
        double new_theta =  atan2(pos(1),pos(0)) +  dt*initial_speed_/radius;
        Eigen::Vector3d p(radius*cos(new_theta), radius*sin(new_theta), pos(2));
        desired_state_->pos() = Eigen::Vector3d(1000,1000, p(2));
        desired_state_->pos() = p;
        Eigen::Vector3d r(p(0),p(1),0);
        Eigen::Vector3d v = (p-pos).normalized() * initial_speed_;
        desired_state_->quat().set(0, 0, atan2(v(1),v(0)));
        return true;
      }
    } // namespace autonomy
  } // namespace scrimmage
