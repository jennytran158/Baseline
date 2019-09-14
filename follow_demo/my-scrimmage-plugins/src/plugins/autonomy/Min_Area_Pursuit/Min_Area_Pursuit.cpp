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

#include <my-scrimmage-plugins/plugins/autonomy/Min_Area_Pursuit/Min_Area_Pursuit.h>
#include <scrimmage/common/RTree.h>
#include <scrimmage/common/Shape.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/math/Angles.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
  scrimmage::autonomy::Min_Area_Pursuit,
  Min_Area_Pursuit_plugin)

  namespace scrimmage {
    namespace autonomy {

      void Min_Area_Pursuit::init(std::map<std::string, std::string> &params) {
        initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);
        attack_view_length_ = sc::get<double>("attack_view_length", params, attack_view_length_);
        auto callback = [&] (scrimmage::MessagePtr<std::tuple<int,int,Eigen::Vector3d>> msg) {
          auto temp = msg->data;
          int i = std::get<0>(temp);
          int follow_id = std::get<1>(temp);
          Eigen::Vector3d pos = std::get<2>(temp);
          if (!attacking_  && i == parent_->id().id()){
            base_sent_pos = pos;
            attack_time = 0;
            attack_end_time = 0;
          }
        };
        subscribe<std::tuple<int,int,Eigen::Vector3d>>("GlobalNetwork", "paths", callback);
        killed_ids_ = advertise("GlobalNetwork", "Killed_IDs",100);
        auto killed_callback = [&] (scrimmage::MessagePtr<int> msg) {
          if (msg->data == follow_id_) {
            attack_time = 0;
            attack_end_time = 0;
            pre_follow_id_ = follow_id_;
            follow_id_ = -1;
          }
        };
        subscribe<int>("GlobalNetwork", "Killed_IDs", killed_callback);
      }

      bool Min_Area_Pursuit::step_autonomy(double t, double dt) {
        Eigen::Vector3d &pos = state_->pos();
        attacking_ = false;
        if (follow_id_ > 0) {
          pursue(t,dt);
        } else {
          std::vector<ID> rtree_neighbors;
          rtree_->neighbors_in_range(state_->pos(), rtree_neighbors, view_length_);
          if (rtree_neighbors.size() > 0) {
            double min_dist = 1e9;
            for (auto it = rtree_neighbors.begin(); it != rtree_neighbors.end();++it){
              if (pre_follow_id_ != it->id() && it->id() != parent_->id().id()) {
                printf("Check %d\n",it->id());
                auto ent = contacts_->at(it->id());
                sc::StatePtr ent_state = ent.state();
                Eigen::Vector3d &ent_pos = ent_state->pos();
                if (ent.id().team_id() != parent_->id().team_id() &&
                  state_->InFieldOfView(*(ent_state), fov_width_angle_, fov_height_angle_)) {
                  double dist = (ent_pos - pos).norm();
                  if (dist <  min_dist) {
                    min_dist = dist;
                    follow_id_ = it->id();
                  }
                }
              }
            }
          }
          if (follow_id_ > 0) {
            pursue(t,dt);
          } else {
            if (base_sent_pos == Eigen::Vector3d(0,0,0)) {
              double radius = 20;
             double new_theta =  atan2(pos(1),pos(0)) +  dt*initial_speed_/radius;
             Eigen::Vector3d p(radius*cos(new_theta), radius*sin(new_theta), pos(2));
             desired_state_->pos() = p;
             Eigen::Vector3d v = (p-pos).normalized();
             desired_state_->quat().set(0, 0, atan2(v(1),v(0)));
            }else {
              desired_state_->pos() = base_sent_pos;
              Eigen::Vector3d v = (base_sent_pos-pos).normalized();
              desired_state_->quat().set(0, 0,atan2(v(1),v(0)));
            }
          }
        }
        //
        // attacking_ = false;
        // // change view_color
        // double radius = 20;
        // double new_theta =  atan2(pos(1),pos(0)) +  dt*initial_speed_/radius;
        // Eigen::Vector3d p(radius*cos(new_theta), radius*sin(new_theta), pos(2));
        // desired_state_->pos() = p;
        // Eigen::Vector3d v = (p-pos).normalized();
        // desired_state_->quat().set(0, 0, atan2(v(1),v(0)));
        //animate heading
        base_sent_pos = Eigen::Vector3d(0,0,0);
        return true;
      }
      void Min_Area_Pursuit::pursue(double t, double dt) {
        int id = follow_id_;
        printf("Pursue %d\n",id);

        Eigen::Vector3d &pos = state_->pos();
        auto ent = contacts_->at(id);
        follow_id_ = id;
        sc::StatePtr ent_state = ent.state();
        Eigen::Vector3d &ent_pos = ent_state->pos();
        Eigen::Vector2d p = (ent_pos - pos).head<2>();
        double dist = p.norm();
        Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000;
        Eigen::Vector3d temp = state_->pos() + (ent_pos - pos).normalized() * rel_pos.norm();
        desired_state_->pos() = Eigen::Vector3d(temp(0) , temp(1),ent_pos(2));
        auto angle = state_->rel_az(ent_pos);
        desired_state_->quat().set(0, 0, state_->quat().yaw() + angle);
        attacking_ = false;


        auto line = sc::shape::make_line(state_->pos(), ent_pos,Eigen::Vector3d(255, 0, 0),0.75);
        line->set_persistent(false);
        line->set_ttl(1);
        draw_shape(line);

        if (dist <= attack_view_length_ &&
          state_->InFieldOfView(*ent_state, attack_fov_width_angle_, attack_fov_height_angle_)) {
            attacking_ = true;
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
              killed_ids_->publish(msg);
            }

            if (attack_time == t) {
              attack_time = t + dt;
            } else {
              attack_time = 0;
            }

            //animate attack field of view
            Eigen::Vector3d attack_color = Eigen::Vector3d(100, 0, 0);
            double attack_fov_width = 2 * (attack_view_length_ / tan((M_PI - attack_fov_width_angle_)/2));
            double attack_fov_height = 2 * (attack_view_length_ / tan((M_PI - attack_fov_height_angle_)/2));
            std::list<Eigen::Vector3d> attack_view_points;
            std::list<Eigen::Vector3d> attack_view_points_transformed;
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
            draw_shape(attack_FOV);
          }
        }
      } // namespace autonomy
    } // namespace scrimmage
