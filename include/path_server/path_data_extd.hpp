/**
 * @file path_data_extd.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Path data extended class, an internal class managed by the path server instance. It is
 * derived from the PathData class which is exposed to the plugins, adding various setters. This set
 * of APIs are NOT exposed to plugin developers.
 * @version 0.1
 * @date 2023-09-08
 *
 * @copyright Copyright (c) 2023 Haoguang Yang
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

#ifndef PATH_DATA_EXTD_HPP_
#define PATH_DATA_EXTD_HPP_

#include <cstdint>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <vector>

#if __has_include("rclcpp/rclcpp.hpp")
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#elif __has_include("ros/ros.h")
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "logging.hpp"
#endif

#include "tf2_ros/buffer.h"

// headers from this package
#include "path_data.hpp"
#include "path_property.hpp"
#include "path_utils.hpp"

namespace planning {

#ifdef RCLCPP__RCLCPP_HPP_
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
#elif defined ROSCPP_ROS_H
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
#endif

/**
 * @brief This class is not exposed to prevent external programs from messing with the data
 * integrety. This class is used internally by the path_server class. In addition to the PathData
 * base class, it adds setters to members.
 */
class PathDataExtd : public PathData {
 public:
#ifdef RCLCPP__RCLCPP_HPP_
  typedef rclcpp::Node* NodePtr;
#elif defined ROSCPP_ROS_H
  typedef ros::NodeHandle* NodePtr;
#endif

  PathDataExtd(NodePtr parent, tf2_ros::Buffer* tf, const std::string& name,
               const std::vector<std::pair<std::string, std::string>>& plugins);

  ~PathDataExtd() {
    for (size_t n = properties_.size(); n > 0; n--) {
      properties_[n - 1]->deactivate();
      properties_[n - 1].release();
    }
    pluginLoader_.release();
  };

  void configure(const int& currentIndex, PathCandidate* globalPath,
                 const std::string& targetFrame);

  bool configureProperties();

  void setCurrentIndex(const int& newCurrentIndex, PathCandidate* globalPath = nullptr);

  void setTargetFrame(const std::string& targetFrame) { targetFrame_ = targetFrame; }

  bool updateProperty(const size_t& which) {
    if (which >= properties_.size()) return false;
    if (!properties_[which]->isActive()) return true;
    RCLCPP_DEBUG(node_->get_logger(), "ON UPDATE -- Path property %s/%s", name_.c_str(),
                 properties_[which]->name().c_str());
    return properties_[which]->update();
  }

  void setActivationStates(const std::vector<bool>& activation) {
    if (activation.size() != properties_.size()) return;
    for (size_t n = 0; n < properties_.size(); n++) {
      if (activation[n]) {
        properties_[n]->activate();
        RCLCPP_INFO(node_->get_logger(), "ACTIVATED -- Path property %s/%s", name_.c_str(),
                    properties_[n]->name().c_str());
      } else {
        properties_[n]->deactivate();
        RCLCPP_INFO(node_->get_logger(), "DEACTIVATED -- Path property %s/%s", name_.c_str(),
                    properties_[n]->name().c_str());
      }
    }
  }

  bool setBonds(const std::vector<const PathData*>& bonds, const std::vector<uint8_t>& prio,
                const bool& reset = true);

  // a template can handle both const and non-const pointer version of setPeers
  template <typename PathDataPtrType>
  void setPeers(const std::vector<PathDataPtrType> peers, const bool& reset = true) {
    if (reset) bindings_.clear();
    bindings_.reserve(bindings_.size() + peers.size());
    for (const auto& i : peers) {
      if (i == this) continue;
      // make them const pointers so we don't mess up our peers.
      bindings_.emplace_back(static_cast<const PathData*>(i), 0);
    }
  }

 protected:
  NodePtr node_;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<pluginlib::ClassLoader<PathProperty>> pluginLoader_;
  std::vector<std::string> propertyNames_;
};

}  // namespace planning

#endif  // PATH_DATA_EXTD_HPP_
