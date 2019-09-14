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

#ifndef INCLUDE_CMAKE_PROJECT_PLUGINS_AUTONOMY_EXAMPLEPLUGIN_EXAMPLEPLUGIN_H_
#define INCLUDE_CMAKE_PROJECT_PLUGINS_AUTONOMY_EXAMPLEPLUGIN_EXAMPLEPLUGIN_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

#include <scrimmage/common/VariableIO.h>

namespace scrimmage {
namespace autonomy {
class ExamplePlugin : public scrimmage::Autonomy {
 public:
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step_autonomy(double t, double dt);
    virtual void velocity_controller(Eigen::Vector3d &v);

 protected:

      int follow_id_ = -1;
      double initial_speed_ = 0;
      double attack_view_length = 0;
      Eigen::Vector3d desired_pos;
      double view_length_ = 200;
      double attack_view_length_ = 30;
      double attack_duration = 3;
      double attack_time = 0;
      double attack_end_time = 0;
      double attack_fov_width_angle_ = M_PI/2;
      double attack_fov_height_angle_ = M_PI/2;
      bool attacking_ = false;
      scrimmage::PublisherPtr killed_ids_;


       bool show_shapes_;
       double max_speed_;

       double w_align_;
       double w_avoid_team_;
       double w_centroid_;
       double w_avoid_nonteam_;
       double w_goal_;

       double fov_el_;
       double fov_az_;
       double comms_range_;
       double minimum_team_range_;
       double minimum_nonteam_range_;
       double sphere_of_influence_;

       Eigen::Vector3d goal_;

       // variable io
       uint8_t io_vel_x_idx_ = 0;
       uint8_t io_vel_y_idx_ = 0;
       uint8_t io_vel_z_idx_ = 0;

       uint8_t io_vel_idx_ = 0;
       uint8_t io_turn_rate_idx_ = 0;
       uint8_t io_pitch_rate_idx_ = 0;

       uint8_t io_heading_idx_  = 0;
       uint8_t io_altitude_idx_  = 0;
       uint8_t io_desired_speed_idx_ = 0;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_CMAKE_PROJECT_PLUGINS_AUTONOMY_EXAMPLEPLUGIN_EXAMPLEPLUGIN_H_
