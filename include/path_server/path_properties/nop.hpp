/**
 * @file nop.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief An example and blank path server plugin
 * @version 0.1
 * @date 2023-09-05
 * 
 * @copyright Copyright (c) 2022-2023 Haoguang Yang
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
 * in compliance with the License. You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express
 * or implied. See the License for the specific language governing permissions and limitations under
 * the License.
 * 
 */

#ifndef PATH_PROPERTY_NOP_HPP
#define PATH_PROPERTY_NOP_HPP

#include "path_server/path_property.hpp"
#include "path_server/path_data.hpp"

namespace planning {

/**
 * @brief This plugin does nothing (No Operation). When called during update, it returns a constant
 * success. This plugin is used as a spacer during interleaving updates to reolve dependencies
 * between bonded paths. It can also be used as a minimal example of how to use the path property
 * plugin.
 */
class NOP : public PathProperty {
 public:
  virtual void configure(NodePtr parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf, PathData* path) override final {
    name_ = name;
    (void)parent;
    (void)tf;
    (void)path;
  };

  virtual bool update() override final { return true; }
};

}  // namespace planning

#endif
