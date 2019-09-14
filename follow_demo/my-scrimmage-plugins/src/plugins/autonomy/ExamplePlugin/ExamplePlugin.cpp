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

#include <my-scrimmage-plugins/plugins/autonomy/ExamplePlugin/ExamplePlugin.h>

#include <scrimmage/common/RTree.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/VariableIO.h>

#include <iostream>
#include <limits>

#include <scrimmage/proto/Shape.pb.h>         // scrimmage_proto::Shape
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Shape.h>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy,
                scrimmage::autonomy::ExamplePlugin,
                ExamplePlugin_plugin)

namespace scrimmage {
namespace autonomy {

// ExamplePlugin::ExamplePlugin() : follow_id_(-1) {
// }

void ExamplePlugin::init(std::map<std::string, std::string> &params) {
  initial_speed_ = sc::get<double>("initial_speed", params, initial_speed_);
  attack_view_length = sc::get<double>("attack_view_length", params, attack_view_length);
  auto callback = [&] (scrimmage::MessagePtr<std::tuple<int,int,Eigen::Vector3d>> msg) {
    auto temp = msg->data;
    int i = std::get<0>(temp);
    int follow_id = std::get<1>(temp);
    Eigen::Vector3d pos = std::get<2>(temp);
    if (!attacking_  && i == parent_->id().id()){
      desired_pos = pos;
      follow_id_ = follow_id;
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
      follow_id_ = -1;
    }
  };
  subscribe<int>("GlobalNetwork", "Killed_IDs", killed_callback);

  show_shapes_ = get("show_shapes", params, true);
  max_speed_ = get<double>("max_speed", params, 21);

  w_align_ = get("align_weight", params, 0.01);
  w_avoid_team_ = get("avoid_team_weight", params, 0.95);
  w_centroid_ = get("centroid_weight", params, 0.05);

  w_avoid_nonteam_ = get("avoid_nonteam_weight", params, 1.0);

  fov_el_ = attack_fov_width_angle_;
  fov_az_ = attack_fov_height_angle_;
  comms_range_ = get("comms_range", params, 1000);

  sphere_of_influence_ = get<double>("sphere_of_influence", params, 10);
  minimum_team_range_ = get<double>("minimum_team_range", params, 5);
  minimum_nonteam_range_ = get<double>("minimum_nonteam_range", params, 10);

  w_goal_ = get<double>("goal_weight", params, 1.0);

  if (get("use_initial_heading", params, false)) {
      Eigen::Vector3d rel_pos = Eigen::Vector3d::UnitX() * 1000000;
      Eigen::Vector3d unit_vector = rel_pos.normalized();
      unit_vector = state_->quat().rotate(unit_vector);
      goal_ = state_->pos() + unit_vector * rel_pos.norm();
  } else {
      std::vector<double> goal_vec;
      if (get_vec<double>("goal", params, " ", goal_vec, 3)) {
          goal_ = vec2eigen(goal_vec);
      }
  }
  goal_ = Eigen::Vector3d(0,0,0);
  printf("Goal: %lf %lf %lf\n", goal_(0),goal_(1),goal_(2));
  io_vel_x_idx_ = vars_.declare(VariableIO::Type::velocity_x, VariableIO::Direction::Out);
  io_vel_y_idx_ = vars_.declare(VariableIO::Type::velocity_y, VariableIO::Direction::Out);
  io_vel_z_idx_ = vars_.declare(VariableIO::Type::velocity_z, VariableIO::Direction::Out);

  io_vel_idx_ = vars_.declare(VariableIO::Type::speed, VariableIO::Direction::Out);
  io_turn_rate_idx_ = vars_.declare(VariableIO::Type::turn_rate, VariableIO::Direction::Out);
  io_pitch_rate_idx_ = vars_.declare(VariableIO::Type::pitch_rate, VariableIO::Direction::Out);

  io_desired_speed_idx_ = vars_.declare(VariableIO::Type::desired_speed, VariableIO::Direction::Out);
  io_heading_idx_ = vars_.declare(VariableIO::Type::desired_heading, VariableIO::Direction::Out);
  io_altitude_idx_ = vars_.declare(VariableIO::Type::desired_altitude, VariableIO::Direction::Out);
}

bool ExamplePlugin::step_autonomy(double t, double dt) {
  printf("Goal: %lf %lf %lf\n", goal_(0),goal_(1),goal_(2));

  // Find neighbors that are within field-of-view and within comms range
  std::vector<ID> rtree_neighbors;
  rtree_->neighbors_in_range(state_->pos(), rtree_neighbors, comms_range_);

  // Remove neighbors that are not within field of view
  for (auto it = rtree_neighbors.begin(); it != rtree_neighbors.end();
       /* no inc */) {

      // Ignore own position / id
      if (it->id() == parent_->id().id()) {
          it = rtree_neighbors.erase(it);
      } else if (state_->InFieldOfView(*(*contacts_)[it->id()].state(), fov_az_, fov_el_)) {
          // The neighbor is "in front"
          ++it;
      } else {
          // The neighbor is "behind." Remove it.
          it = rtree_neighbors.erase(it);
      }
  }

  // move-to-goal behavior
  Eigen::Vector3d v_goal = (goal_ - state_->pos()).normalized();

  // Steer to avoid local neighbors
  // Align with neighbors
  // Cohesion: move towards average position of neighbors
  // (i.e., Find centroid of neighbors)
  double heading = 0;
  Eigen::Vector3d align(0, 0, 0);
  Eigen::Vector3d centroid(0, 0, 0);

  // Compute repulsion vector from each robot (team/nonteam)
  std::vector<Eigen::Vector3d> O_team_vecs;
  std::vector<Eigen::Vector3d> O_nonteam_vecs;

  for (ID id : rtree_neighbors) {
      bool is_team = (id.team_id() == parent_->id().team_id());

      StatePtr other_state = (*contacts_)[id.id()].state();

      // Calculate vector pointing from own position to other
      Eigen::Vector3d diff = other_state->pos() - state_->pos();
      double dist = diff.norm();

      // Calculate magnitude of repulsion vector
      double min_range = is_team ? minimum_team_range_ : minimum_nonteam_range_;
      double O_mag = 0;
      if (dist > sphere_of_influence_) {
          O_mag = 0;
      } else if (min_range < dist && dist <= sphere_of_influence_) {
          O_mag = (sphere_of_influence_ - dist) /
              (sphere_of_influence_ - min_range);
      } else if (dist <= min_range) {
          O_mag = 1e10;
      }

      // Calculate repulsion vector
      Eigen::Vector3d O_dir = - O_mag * diff.normalized();
      if (is_team) {
          O_team_vecs.push_back(O_dir);
      } else {
          O_nonteam_vecs.push_back(O_dir);
      }

      // Calculate centroid of team members and heading alignment
      if (is_team) {
          centroid = centroid + other_state->pos();
          align += other_state->vel().normalized();
          heading += other_state->quat().yaw();
      }
  }

  Eigen::Vector3d align_vec(0, 0, 0);
  if (rtree_neighbors.size() > 0) {
      centroid = centroid / static_cast<double>(rtree_neighbors.size());
      align = align / static_cast<double>(rtree_neighbors.size());
      heading /= static_cast<double>(rtree_neighbors.size());
      align_vec << cos(heading), sin(heading), 0;
  }

  // Make sure alignment vector is well-behaved
  align = align_vec; // TODO
  Eigen::Vector3d v_align_normed = align.normalized();
  double v_align_norm = align.norm();
  if (v_align_normed.hasNaN()) {
      v_align_normed = Eigen::Vector3d::Zero();
      v_align_norm = 0;
  }

  // Normalize each team repulsion vector and sum
  Eigen::Vector3d O_team_vec(0, 0, 0);
  for (Eigen::Vector3d v : O_team_vecs) {
      if (v.hasNaN()) {
          continue; // ignore misbehaved vectors
      }
      O_team_vec += v;
  }

  // Normalize each nonteam repulsion vector and sum
  Eigen::Vector3d O_nonteam_vec(0, 0, 0);
  for (Eigen::Vector3d v : O_nonteam_vecs) {
      if (v.hasNaN()) {
          continue; // ignore misbehaved vectors
      }
      O_nonteam_vec += v;
  }

  // Apply gains to independent behaviors
  Eigen::Vector3d v_goal_w_gain = v_goal * w_goal_;
  Eigen::Vector3d O_team_vec_w_gain = O_team_vec * w_avoid_team_;
  Eigen::Vector3d O_nonteam_vec_w_gain = O_nonteam_vec * w_avoid_nonteam_;
  Eigen::Vector3d v_centroid_w_gain = (centroid - state_->pos()).normalized() * w_centroid_;
  Eigen::Vector3d v_align_w_gain = v_align_normed * w_align_;

  double sum_norms = v_goal_w_gain.norm() + O_team_vec_w_gain.norm() +
      O_nonteam_vec_w_gain.norm() + v_centroid_w_gain.norm() +
      v_align_norm;

  Eigen::Vector3d v_sum = (v_goal_w_gain + O_team_vec_w_gain +
                           O_nonteam_vec_w_gain + v_centroid_w_gain +
                           v_align_w_gain) / sum_norms;

  // Scale velocity to max speed:
  Eigen::Vector3d vel_result = v_sum * max_speed_;
  Eigen::Vector3d v = vel_result;

  if (rtree_neighbors.size() > 0) {

      velocity_controller(vel_result);

      if (show_shapes_) {
          auto sphere = sc::shape::make_sphere(state_->pos(), sphere_of_influence_, Eigen::Vector3d(0, 255, 0),0.1);
          draw_shape(sphere);

          // Draw resultant vector:
          auto line = sc::shape::make_line(state_->pos(), vel_result + state_->pos(),Eigen::Vector3d(255, 255, 0),0.75);
          draw_shape(line);
      }
  } else {
    v = v_goal;
      velocity_controller(v_goal);
  }
  desired_state_->pos() = goal_;
  // desired_state_->pos() = Eigen::Vector3d(1000 ,1000, desired_pos(2));
  desired_state_->quat().set(0, 0,atan2(v(1),v(0)));
  return true;
}

void ExamplePlugin::velocity_controller(Eigen::Vector3d &v) {
    // Convert to spherical coordinates:
    double desired_heading = atan2(v(1), v(0));
    double desired_pitch = atan2(v(2), v.head<2>().norm());

    vars_.output(io_vel_idx_, max_speed_);
    vars_.output(io_turn_rate_idx_, desired_heading - state_->quat().yaw());
    vars_.output(io_pitch_rate_idx_, desired_pitch + state_->quat().pitch());

    vars_.output(io_heading_idx_, desired_heading);
    vars_.output(io_altitude_idx_, v(2));
    vars_.output(io_desired_speed_idx_, max_speed_);

    double norm = v.norm();
    double ratio = (max_speed_ / 2) / std::max(norm, 1.0);
    if (norm != 0 && ratio < 1) {
        v *= ratio;
    }

    vars_.output(io_vel_x_idx_, v(0));
    vars_.output(io_vel_y_idx_, v(1));
    vars_.output(io_vel_z_idx_, v(2));
}
} // namespace autonomy
} // namespace scrimmage
