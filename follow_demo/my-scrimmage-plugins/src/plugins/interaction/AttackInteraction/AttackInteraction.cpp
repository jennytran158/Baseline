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

#include <my-scrimmage-plugins/plugins/interaction/AttackInteraction/AttackInteraction.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>

#include <memory>
#include <limits>
#include <iostream>
using std::vector;
using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
  scrimmage::interaction::AttackInteraction,
  AttackInteraction_plugin)

  namespace scrimmage {
    namespace interaction {

      AttackInteraction::AttackInteraction() {
      }

      bool AttackInteraction::init(std::map<std::string, std::string> &mission_params,
        std::map<std::string, std::string> &plugin_params) {
          // dead_ids_ = advertise("GlobalNetwork", "Dead_IDs",100);
          auto callback = [&] (scrimmage::MessagePtr<int> msg) {
            killed_ids_.push_back(msg->data);
          };
          subscribe<int>("GlobalNetwork", "Killed_IDs", callback);
          return true;
        }


        bool AttackInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
          double t, double dt) {
            for (EntityPtr ent1 : ents) {
              int id = ent1->id().id();
              if (!killed_ids_.empty()) {
                auto ptr = std::find(killed_ids_.begin(),killed_ids_.end(), id);
                if (*ptr == id) {
                  printf("id: %d killed\n",id);
                  ent1->set_health_points(-1e9);
                }
              }
            }
            return true;
          }
        } // namespace interaction
      } // namespace scrimmage
