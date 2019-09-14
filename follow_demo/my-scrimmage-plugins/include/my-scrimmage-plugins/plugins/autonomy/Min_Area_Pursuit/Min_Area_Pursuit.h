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

#ifndef INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_MIN_AREA_PURSUIT_MIN_AREA_PURSUIT_H_
#define INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_MIN_AREA_PURSUIT_MIN_AREA_PURSUIT_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {
class Min_Area_Pursuit : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    void pursue(double t, double dt);

 protected:
    int pre_follow_id_ = -1;
    int follow_id_ = -1;
    double initial_speed_ = 0;
    Eigen::Vector3d base_sent_pos;
    double view_length_ = 200;
    double attack_view_length_ = 30;
    double attack_duration = 3;
    double attack_time = 0;
    double attack_end_time = 0;
    double attack_fov_width_angle_ = M_PI/2;
    double attack_fov_height_angle_ = M_PI/2;
    double fov_width_angle_ = M_PI/2;
    double fov_height_angle_ = M_PI/2;
    bool attacking_ = false;
    scrimmage::PublisherPtr killed_ids_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_MIN_AREA_PURSUIT_MIN_AREA_PURSUIT_H_
