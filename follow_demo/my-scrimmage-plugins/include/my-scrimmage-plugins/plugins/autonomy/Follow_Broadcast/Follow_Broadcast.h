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

#ifndef INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_FOLLOW_BROADCAST_FOLLOW_BROADCAST_H_
#define INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_FOLLOW_BROADCAST_FOLLOW_BROADCAST_H_
#include <scrimmage/autonomy/Autonomy.h>

#include <string>
#include <map>

#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/common/Shape.h>
namespace scrimmage {
namespace autonomy {
class Follow_Broadcast : public scrimmage::Autonomy {
 public:
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;

 protected:
    int follow_id_ = -1;
    double max_speed_ = 0;
    double attack_view_length_ = 0;
    double view_length_ = 0;
    scrimmage_proto::ShapePtr attack_sphere_;
    scrimmage::PublisherPtr enemy_ids_;
};
} // namespace autonomy
} // namespace scrimmage
#endif // INCLUDE_MY_SCRIMMAGE_PLUGINS_PLUGINS_AUTONOMY_FOLLOW_BROADCAST_FOLLOW_BROADCAST_H_
