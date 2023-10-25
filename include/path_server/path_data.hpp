/**
 * @file path_data.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Data structure associated with a path candidate. The scope covers the closest waypoint to
 * the robot, the peered/bonded paths with the same context, and path properties. This is the
 * user-side API which exposes mainly getters, to prevent users from messing with the protected
 * data.
 * @version 0.1
 * @date 2023-07-12
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

#ifndef PATH_DATA_HPP_
#define PATH_DATA_HPP_

#include <cstdint>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <string_view>
#include <vector>

#if __has_include("rclcpp/rclcpp.hpp")
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#elif __has_include("ros/ros.h")
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#endif

// headers from this package
#include "path_property.hpp"
#include "type_map.hpp"

namespace planning {

#ifdef RCLCPP__RCLCPP_HPP_
using geometry_msgs::msg::PoseStamped;
#elif defined ROSCPP_ROS_H
using geometry_msgs::PoseStamped;
#endif

/**
 * @brief The PathCandidate struct resembles nav_msgs::msg::Path, but with a few additional
 * properties and convenience methods.
 */
struct PathCandidate {
  const std::string frame_id;
  unsigned long version;
  bool isClosedPath;
  std::vector<PoseStamped> poses;

  PathCandidate(const std::string& frame_id) : frame_id(frame_id) {}

  // shortcuts of std::vector operators
  size_t size() const { return poses.size(); }
  std::vector<PoseStamped>::const_iterator cbegin() const { return poses.cbegin(); }
  std::vector<PoseStamped>::const_iterator cend() const { return poses.cend(); }
  std::vector<PoseStamped>::iterator begin() { return poses.begin(); }
  std::vector<PoseStamped>::iterator end() { return poses.end(); }
  std::vector<PoseStamped>::const_reverse_iterator crbegin() const { return poses.crbegin(); }
  std::vector<PoseStamped>::const_reverse_iterator crend() const { return poses.crend(); }
  std::vector<PoseStamped>::reverse_iterator rbegin() { return poses.rbegin(); }
  std::vector<PoseStamped>::reverse_iterator rend() { return poses.rend(); }
  PoseStamped back() { return poses.back(); }
  PoseStamped front() { return poses.front(); }
};

/**
 * @brief A base class of Path Data. This class is exposed to plugins, and only has read-only
 * methods (getters) to most protected members. Setters are not involved.
 */
class PathData {
 public:
  /**
   * @brief Construct a new Path Data object
   *
   * @param name human-readable identifier of the path data instance.
   */
  PathData(const std::string& name) : name_(name){};

  ~PathData() = default;

  /**
   * @brief Get the Property object with a specific name from the managed members.
   *
   * @tparam Property derived property type
   * @param propertyStr fully-qualified type name of the property to find
   * @return Property* pointer to the desired property
   */
  template <typename Property>
  Property* getProperty(const std::string_view& propertyStr) const {
    auto it = properties_.find(propertyStr);
    if (it == properties_.end()) return nullptr;
    return dynamic_cast<Property*>((*it).get());
  }

  /**
   * @brief static version to get properties of derived class. needs a fully-qualified type (include
   * namespace with no alias)
   */
  template <typename Property>
  Property* _staticGetProperty(const std::string_view& propertyStr) const {
    auto it = properties_.find(propertyStr);
    if (it == properties_.end()) return nullptr;
    return static_cast<Property*>((*it).get());
  }

  /**
   * @brief macro to call the _staticGetProperty method, passing in a fully-qualified type of the
   * property to find. The macro automatically fills in both the template type and the type name
   * string.
   */
#define GET_PROPERTY(type_name) _staticGetProperty<type_name>(#type_name)

  /**
   * @brief MOVEs the property instance unique pointer to the property list. The list is a
   * hash map that supports indexing both by name and by order.
   *
   * @tparam Property Value type of the list.
   * @param typeStr Value type (fully-qualified name) in string format
   * @param p Shared pointer to the property instance. The original object will be swapped after
   * calling this method.
   */
  template <typename Property>
  void provideProperty(const std::string& typeStr, pluginlib::UniquePtr<Property> p) {
    properties_.put(typeStr, std::move(p));
  }

  /**
   * @brief macro to add new properties. The macro automatically fills in both the template type and
   * the type name string.
   */
#define PROVIDE_PROPERTY(type_name, p) provideProperty<type_name>(#type_name, p)

  /**
   * @brief Get the Current Index object, which indicates the closest coordinate being
   * `globalPath_.poses[currInd_]`.
   *
   * @return const int& const reference to the currInd_ member.
   */
  const int& getCurrentIndex() const { return currInd_; }

  /**
   * @brief Get the Previous Index object. This was the currInd_ before last update.
   *
   * @return const int& const reference to the prevInd_ member.
   */
  const int& getPreviousIndex() const { return prevInd_; }

  /**
   * @brief Get the Current Index Delta object. This object indicates the difference `currInd_ -
   * prevInd`.
   *
   * @return const int& difference in currInd_ from previous update to this update.
   */
  const int& getCurrentIndexDelta() const { return currIndDelta_; }

  /**
   * @brief Get the entire path.
   *
   * @return PathCandidate* The entire path expressed in a transformable reference frame.
   */
  PathCandidate* getGlobalPath() const { return globalPath_; }

  /**
   * @brief Whether the path is closed (i.e. the next point from the end is the first point).
   *
   * @return true the path is closed
   * @return false the path is open
   */
  const bool& isClosedPath() const { return globalPath_->isClosedPath; }

  /**
   * @brief Get the target frame from which the closesest point is evaluated.
   *
   * @return const std::string& target frame ID
   */
  const std::string& getTargetFrame() const { return targetFrame_; }

  /**
   * @brief Get the closest sample point on the entire path from the robot (targetFrame_).
   *
   * @return const PoseStamped& the closest sample point on the path
   */
  const PoseStamped& getClosestWaypoint() const { return closestWaypoint_; }

  /**
   * @brief Get the path property plugin activation states as a vector of bool.
   *
   * @return std::vector<bool> a vector describing whether a property of this path is active.
   * Elements are in provision order of the properties.
   */
  std::vector<bool> getActivationStates() const {
    std::vector<bool> ret;
    ret.reserve(properties_.size());
    for (size_t n = 0; n < properties_.size(); n++) {
      ret.emplace_back(properties_[n]->isActive());
    }
    return ret;
  }

  /**
   * @brief Peers are path data with same contexts (e.g. depicting the same path). Peers are
   * distinguished by their names. This method returns the vector of peers of this path.
   */
  std::vector<const PathData*> getPeers() const {
    std::vector<const PathData*> ret;
    ret.reserve(bindings_.size());
    for (const auto& b : bindings_) {
      ret.emplace_back(b.first);
    }
    return ret;
  }

  /**
   * @brief Bonds are peers with synchronized contents. Bonded path data has same context and same
   * size of globalPath_. The waypoints in their globalPath_ are synchronized and one-to-one
   * corresponded. Bonds are updated with a leader-follower pattern, where the peer with highest
   * priority (uint8_t) are updated first. The sort option sorts the output in decreasing priority.
   * This method returns the bonds connected to this path.
   */
  std::vector<std::pair<const PathData*, uint8_t>> getBonds(const bool& sort = false) const {
    std::vector<std::pair<const PathData*, uint8_t>> ret;
    ret.reserve(bindings_.size());
    for (const auto& b : bindings_) {
      if (b.second) ret.emplace_back(b.first, b.second);
    }
    if (sort) {
      std::sort(ret.begin(), ret.end(),
                [](const auto& lhs, const auto& rhs) { return lhs.second > rhs.second; });
    }
    return ret;
  }

  /**
   * @brief In a bond, the leader is the associated path with the highest update priority. As the
   * path server periodiccally updates, the leader is updated first in index and properties. i.e.
   * the leader will be the path[0] in the following update pattern:
   * ```
   * path[0].property[0].update(), path[1].property[0].update(), path[2].property[0].update, ...,
   * path[0].property[1].update(), path[1].property[1].update(), ...
   * ```
   *
   * @return const PathData* pointer to the leader path data instance. If myself is the leader, it
   * returns a pointer to `this`.
   */
  const PathData* getLeader() const { return leaderPath_; }

  /**
   * @brief In a bond, the leader is the associated path with the highest update priority. This
   * method determins if this instance is a leader in a bond.
   *
   * @return true This instance is a leader in a bond, or there is no bond.
   * @return false This instance is associated to another leader path.
   */
  bool isLeader() const { return leaderPath_ == nullptr || leaderPath_ == this; }

  /**
   * @brief Get the human-readable name of this path data instance.
   *
   * @return const std::string& the human-readable identifier of this path data instance.
   */
  const std::string& getName() const { return name_; }

  /**
   * @brief Get the number of properties associated to this path.
   *
   * @return size_t number of properties.
   */
  size_t getNumProperties() const { return properties_.size(); }

 protected:
  TypeMap<pluginlib::UniquePtr<PathProperty>> properties_{};
  // A data structure to store peers and bonds
  std::vector<std::pair<const PathData*, uint8_t>> bindings_{};

  std::string name_;

  int currInd_ = -1;
  int prevInd_ = -1;
  int currIndDelta_ = 0;
  // global properties
  PathCandidate* globalPath_ = nullptr;
  const PathData* leaderPath_ = nullptr;
  std::string targetFrame_;
  PoseStamped closestWaypoint_;
};

}  // namespace planning

#endif  // PATH_DATA_HPP_
