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

#include <my-scrimmage-plugins/plugins/autonomy/Follow_Broadcast/Follow_Broadcast.h>
#include <scrimmage/common/Shape.h>

#include <vector>
#include <list>
using namespace std;
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <iostream>
#include <limits>
#include <tuple>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::Follow_Broadcast,
  Follow_Broadcast_plugin)

  namespace scrimmage {
    namespace autonomy {

      void Follow_Broadcast::init(std::map<std::string, std::string> &params) {
        attack_view_length_ = sc::get<double>("attack_view_length", params, attack_view_length_);
        view_length_ = sc::get<double>("view_length", params, view_length_);
        max_speed_ = sc::get<double>("max_speed", params, max_speed_);
        enemy_ids_ = advertise("GlobalNetwork", "Enemy_IDs",2);
      }

      bool Follow_Broadcast::step_autonomy(double t, double dt) {
        Eigen::Vector3d attack_color = Eigen::Vector3d(100, 0, 0);
        Eigen::Vector3d view_color = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d &pos = state_->pos();
        Eigen::Vector3d &vel = state_->vel();

        //animate field of view
        double fov_width_angle = M_PI/2;
        double fov_width = 2 * (view_length_ / tan((M_PI - fov_width_angle)/2));
        double fov_height_angle = M_PI/2;
        double fov_height = 2 * (view_length_ / tan((M_PI - fov_width_angle)/2));;
        list<Eigen::Vector3d> points;
        list<Eigen::Vector3d> points_transformed;
        points.push_back(Eigen::Vector3d(0,0,0));
        points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2, fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2,  -fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_, fov_width/2, -fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
        points.push_back(Eigen::Vector3d(0,0,0));

        //animate attack view
        list<Eigen::Vector3d> attack_points;
        list<Eigen::Vector3d> attack_points_transformed;
        double attack_fov_width = 2 * (view_length_ / tan((M_PI - fov_width_angle)/2));
        double attack_fov_height = 2 * (view_length_ / tan((M_PI - fov_width_angle)/2));;
        attack_points.push_back(Eigen::Vector3d(0,0,0));
        attack_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
        attack_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2, attack_fov_height/2));
        attack_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2,  -attack_fov_height/2));
        attack_points.push_back(Eigen::Vector3d(-attack_view_length_, attack_fov_width/2, -attack_fov_height/2));
        attack_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
        attack_points.push_back(Eigen::Vector3d(0,0,0));

        // printf("fov_width_angle:%lf fov_height_angle:%lf\n", fov_width_angle,fov_height_angle);
        // printf("%lf %lf\n", fov_width,fov_height);


        follow_id_ = -1;
        for (auto it = contacts_->begin(); it != contacts_->end(); it++) {

          // Skip if this contact is on the same team
          if (it->second.id().team_id() == parent_->id().team_id()) {
            continue;
          }

          // Calculate distance to entity
          double dist = (it->second.state()->pos() - state_->pos()).norm();

          if (dist < view_length_ && state_->InFieldOfView(*(it->second.state()), fov_width_angle, fov_height_angle)) {
            follow_id_ = it->first;
          }
        }

        // Head toward entity on other team
        if (contacts_->count(follow_id_) > 0) {
          auto ent = contacts_->at(follow_id_);
          // Get a reference to the entity's state.
          sc::StatePtr ent_state = ent.state();
          // Calculate the required heading to follow the other entity
          Eigen::Vector3d &ent_pos = ent_state->pos();
          Eigen::Vector3d r = (ent_pos - pos).normalized();
          auto quat = desired_state_->quat();
          desired_state_->pos() = Eigen::Vector3d(1000 ,1000, ent_pos(2));
          // change view_color
          view_color = Eigen::Vector3d(255, 0, 0);
          auto angle = state_->rel_az(ent_pos) + dt*ent_state->ang_vel()(2);
          desired_state_->quat().set(0, 0, state_->quat().yaw() + angle);
          //broadcast enemy positions
          auto msg = std::make_shared<sc::Message<int>>();
          msg->data = follow_id_;
          enemy_ids_->publish(msg);
          //animate heading
          auto arrow = sc::shape::make_arrow(pos,desired_state_->quat(), 10,attack_color,0.1);
          arrow->set_persistent(false);
          arrow->set_ttl(1);
          for (auto iter = points.begin(); iter != points.end();iter++){
            points_transformed.push_back(pos - (state_->quat().normalized() * *iter));
          }
          auto FOV = sc::shape::make_polygon(points_transformed, view_color,0.2);
          FOV->set_persistent(false);
          FOV->set_ttl(1);
          sc::set(FOV->mutable_color(), view_color);
          draw_shape(FOV);
          draw_shape(arrow);
        } else {
          desired_state_->pos() = pos;
          desired_state_->quat().set(0, 0, state_->quat().yaw());
        }
        if (attack_sphere_ == nullptr) {
          attack_sphere_ = sc::shape::make_sphere(state_->pos(), attack_view_length_, attack_color,0.25);
        }
        sc::set(attack_sphere_->mutable_sphere()->mutable_center(), pos);
        sc::set(attack_sphere_->mutable_color(), attack_color);
        draw_shape(attack_sphere_);

        return true;
      }
    } // namespace autonomy
  } // namespace scrimmage
