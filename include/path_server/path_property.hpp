/**
 * @file path_property.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Base class of the path server plugin. Each derived plugin must inherit from this class
 * (and include path_data.hpp if they are accessing the global path), to be properly loaded into,
 * and update with the path server node.
 * @version 0.1
 * @date 2023-09-07
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

#ifndef PATH_PROPERTY_HPP_
#define PATH_PROPERTY_HPP_

#if __has_include("rclcpp/rclcpp.hpp")
#include "rclcpp/rclcpp.hpp"
#elif __has_include("ros/ros.h")
#include "ros/ros.h"
#endif

#include "tf2_ros/buffer.h"

namespace planning {

/**
 * @brief Forward declaration of PathData as an abstract class, such that the path property base
 * class can use it as a black box. Plugin authors need to include path_data.hpp separately in their
 * plugins if they want to access the global path.
 */
class PathData;

/**
 * @brief Base class for each path property plugin.
 */
class PathProperty {
 public:
#ifdef RCLCPP__RCLCPP_HPP_
  typedef rclcpp::Node* NodePtr;
#elif defined ROSCPP_ROS_H
  typedef ros::NodeHandle* NodePtr;
#endif

  /**
   * @brief Destroy the Path Property object. The property plugin may define its own destruction
   * behavior by overriding this method.
   */
  virtual ~PathProperty() = default;

  /**
   * @brief Configure the path property plugin upon startup. This function is called by the path
   * server node after all the paths are loaded, initialized, and the paths have been bonded by
   * context. The property plugin must provide an overriding method to this function.
   *
   * @param parent handle to the parent ROS2 node
   * @param name name of this property
   * @param tf pointer to a pre-configured TF buffer
   * @param path pointer to the path data instance, which is shared by multiple path properties. The
   * path data instance manages the storage and fault tolerance of a single path, on which this
   * property will be evaluated. The path data instance also maintains the closest sample point on
   * the managed path, as observed from the robot.
   */
  virtual void configure(NodePtr parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf, PathData* path) = 0;

  /**
   * @brief If this property is active, this method is called in each updating period of the path
   * server node. This method is invoked after the PathData has been updated. When multiple
   * properties are provided for the same path, their update() method is invoked at the order of
   * provision. When multiple paths are bonded by context, the update() method of each property of
   * each path are invoked in an interleaving order, i.e.:
   * ```
   * path[0].property[0].update(), path[1].property[0].update(), path[2].property[0].update, ...,
   * path[0].property[1].update(), path[1].property[1].update(), ...
   * ```
   *
   * @return true The update is successful
   * @return false The update has failed
   */
  virtual bool update() { return false; }

  /**
   * @brief Getter for the name of this property.
   *
   * @return const std::string& reference to the given name.
   */
  const std::string& name() const { return name_; }

  /**
   * @brief Obtain the activation status of this path property.
   *
   * @return true the path property is currently active for the associated path data.
   * @return false the path property is currently inactive for the associated path data.
   */
  const bool& isActive() const { return active_; }

  /**
   * @brief Activate the path property for the associated path data. This includes setting up
   * publishers and subscribers, etc.
   */
  virtual void activate() { active_ = true; }

  /**
   * @brief Deactivate the path property for the associated path data. This includes tearing down
   * publishers and subscribers, etc.
   */
  virtual void deactivate() { active_ = false; }

 protected:
  /**
   * @brief Construct a new Path Property object. The constructor MUST have an empty input list for
   * the plugin to load. Leave it as the default constructor if unused.
   */
  PathProperty() = default;

  std::string name_ = "";
  bool active_ = false;
};

}  // namespace planning

#endif  // PATH_PROPERTY_HPP_
