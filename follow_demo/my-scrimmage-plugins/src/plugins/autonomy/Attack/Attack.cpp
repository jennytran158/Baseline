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

#include <my-scrimmage-plugins/plugins/autonomy/Attack/Attack.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Shape.h>
#include <iostream>
#include <limits>
using std::list;
using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::Attack,
  Attack_plugin)

  namespace scrimmage {
    namespace autonomy {
      double rnd() {return double(rand())/RAND_MAX;}
      void Attack::init(std::map<std::string, std::string> &params) {
        attack_view_length_ = sc::get<double>("attack_view_length", params, attack_view_length_);
        view_length_ = sc::get<double>("view_length", params, view_length_);
        found_enemy_ids_ = advertise("GlobalNetwork", "Enemy_IDs",100);
        attacked_ids_ = advertise("GlobalNetwork", "Attacked_IDs",100);
      }

      bool Attack::step_autonomy(double t, double dt) {
        Eigen::Vector3d &pos = state_->pos();
        Eigen::Vector3d &vel = state_->vel();

        //animate field of view
        double fov_width_angle = M_PI/2;
        double fov_width = 2 * (view_length_ / tan((M_PI - fov_width_angle)/2));
        double fov_height_angle = M_PI/2;
        double fov_height = 2 * (view_length_ / tan((M_PI - fov_width_angle)/2));
        list<Eigen::Vector3d> points;
        list<Eigen::Vector3d> points_transformed;
        points.push_back(Eigen::Vector3d(0,0,0));
        points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2, fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_, -fov_width/2,  -fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_, fov_width/2, -fov_height/2));
        points.push_back(Eigen::Vector3d(-view_length_,  fov_width/2, fov_height/2));
        points.push_back(Eigen::Vector3d(0,0,0));
        for (auto iter = points.begin(); iter != points.end();iter++){
          points_transformed.push_back(pos - (state_->quat().normalized() * *iter));
        }
        auto FOV = sc::shape::make_polygon(points_transformed,  Eigen::Vector3d(0, 0, 0),0.1);
        FOV->set_persistent(false);
        FOV->set_ttl(1);

        //animate attack field of view
        double attack_fov_width_angle = M_PI/2;
        double attack_fov_height_angle = M_PI/2;
        double attack_fov_width = 2 * (attack_view_length_ / tan((M_PI - attack_fov_width_angle)/2));
        double attack_fov_height = 2 * (attack_view_length_ / tan((M_PI - attack_fov_height_angle)/2));
        list<Eigen::Vector3d> attack_view_points;
        list<Eigen::Vector3d> attack_view_points_transformed;
        attack_view_points.push_back(Eigen::Vector3d(0,0,0));
        attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
        attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2, attack_fov_height/2));
        attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, -attack_fov_width/2,  -attack_fov_height/2));
        attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_, attack_fov_width/2, -attack_fov_height/2));
        attack_view_points.push_back(Eigen::Vector3d(-attack_view_length_,  attack_fov_width/2, attack_fov_height/2));
        attack_view_points.push_back(Eigen::Vector3d(0,0,0));
        for (auto iter = attack_view_points.begin(); iter != attack_view_points.end();iter++){
          attack_view_points_transformed.push_back(pos - (state_->quat().normalized() * *iter));
        }
        auto attack_FOV = sc::shape::make_polygon(attack_view_points_transformed, Eigen::Vector3d(255, 0, 0),0.1);
        attack_FOV->set_persistent(false);
        attack_FOV->set_ttl(1);
        auto arrow = sc::shape::make_arrow(pos,state_->quat(), 7,Eigen::Vector3d(0, 255, 255),0.1);
        arrow->set_persistent(false);
        arrow->set_ttl(1);
        draw_shape(arrow);
        if (current_mode == patrolling) {
          desired_state_->pos() = pos;
          desired_state_->quat().set(0, 0, state_->quat().yaw() + dt);
          follow_id_ = -1;
          for (auto it = contacts_->begin(); it != contacts_->end(); it++) {
            // Skip if this contact is on the same team
            if (it->second.id().team_id() == parent_->id().team_id()) {
              continue;
            }
            // Calculate distance to entity
            double dist = (it->second.state()->pos() - state_->pos()).norm();
            double min_dist = 10000;
            if (state_->InFieldOfView(*(it->second.state()), fov_width_angle, fov_height_angle)) {
              if (dist < view_length_){
                follow_id_ = it->first;
                current_mode = tail_following;
                //broadcast enemy positions
                auto msg = std::make_shared<sc::Message<int>>();
                msg->data = follow_id_;
                found_enemy_ids_->publish(msg);
                if (state_->InFieldOfView(*(it->second.state()), attack_fov_width_angle, attack_fov_height_angle) &&
                dist < attack_view_length_) {
                  current_mode = attacking;
                }
                if (follow_id_ == pre_follow_id_) {
                  break;
                }
              }
            }
          }
          // draw_shape(FOV);
        }
        if (current_mode == tail_following) {
          pre_follow_id_ = follow_id_;
          auto ent = contacts_->at(follow_id_);
          sc::StatePtr ent_state = ent.state();
          Eigen::Vector3d &ent_pos = ent_state->pos();
          auto angle = state_->rel_az(ent_pos + dt*ent_state->vel());
          desired_state_->pos() = Eigen::Vector3d(ent_pos(0), ent_pos(1),ent_pos(2));;

          desired_state_->quat().set(0, 0, state_->quat().yaw() + angle);
          draw_shape(FOV);
          current_mode = patrolling;

          auto arrow = sc::shape::make_arrow(pos,desired_state_->quat(), 7,Eigen::Vector3d(0, 255, 0),0.1);
          arrow->set_persistent(false);
          arrow->set_ttl(1);
          draw_shape(arrow);
          auto dot = sc::shape::make_sphere(ent_pos, 1, Eigen::Vector3d(255, 0, 0),1);
          dot->set_persistent(false);
          dot->set_ttl(1);
          draw_shape(dot);
        }

        if (current_mode == attacking) {
          auto ent = contacts_->at(follow_id_);
          // Get a reference to the entity's state.
          sc::StatePtr ent_state = ent.state();
          // Calculate the required heading to follow the entity
          Eigen::Vector3d &ent_pos = ent_state->pos();
          desired_state_->pos() = Eigen::Vector3d(ent_pos(0) , ent_pos(1),ent_pos(2) + attack_fov_height/2);
          // desired_state_->pos() = ent_pos;
          auto angle = state_->rel_az(ent_pos + dt*ent_state->vel());
          desired_state_->quat().set(0, 0, state_->quat().yaw() + angle);
          if (attack_time == 0){
            attack_time = t;
            attack_end_time = attack_time + dt * attack_duration;
          }
          if (attack_time >= attack_end_time){
            attack_time = 0;
            attack_end_time = 0;
            //broadcast attacked ids
            auto msg = std::make_shared<sc::Message<int>>();
            msg->data = follow_id_;
            attacked_ids_->publish(msg);
          }
          if (attack_time == t) {
            attack_time = t + dt;
          } else {
            attack_time = 0;
          }
          current_mode = patrolling;
          draw_shape(attack_FOV);

          //animate heading

          auto arrow = sc::shape::make_arrow(pos,desired_state_->quat(), 7,Eigen::Vector3d(255, 0, 0),0.1);
          arrow->set_persistent(false);
          arrow->set_ttl(1);
          draw_shape(arrow);

          auto dot = sc::shape::make_sphere(ent_pos, 0.5, Eigen::Vector3d(255, 0, 0),1);
          dot->set_persistent(false);
          dot->set_ttl(1);
          draw_shape(dot);
        }


        return true;
      }
    } // namespace autonomy
  } // namespace scrimmage
