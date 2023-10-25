/**
 * @file path_data_extd.cpp
 * @author Haoguang Yang (yang1510@purdue.edu)
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

#include "path_server/path_data_extd.hpp"

namespace planning {

PathDataExtd::PathDataExtd(
#ifdef RCLCPP__RCLCPP_HPP_
    rclcpp::Node* parent,
#elif defined ROSCPP_ROS_H
    ros::NodeHandle* parent,
#endif
    tf2_ros::Buffer* tf, const std::string& name,
    const std::vector<std::pair<std::string, std::string>>& plugins)
    : PathData{name},
      node_{parent},
      tfBuffer_{tf},
      pluginLoader_{std::make_unique<pluginlib::ClassLoader<PathProperty>>(
          "path_server_overhaul", "planning::PathProperty")} {
  propertyNames_.reserve(plugins.size());
  for (const auto& i : plugins) {
    try {
      // load the plugin
      auto inst = pluginLoader_->createUniqueInstance(i.second);
      provideProperty<PathProperty>(i.second, std::move(inst));
      // store a list of identifier names for configuration later on
      propertyNames_.push_back(i.first);
    } catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(parent->get_logger(),
                   "FAILED -- Path property %s/%s in PathDataExtd() Reason: %s", name.c_str(),
                   i.first.c_str(), ex.what());
    }
  }
}

void PathDataExtd::configure(const int& currentIndex, PathCandidate* globalPath,
                             const std::string& targetFrame) {
  globalPath_ = globalPath;
  currInd_ = currentIndex;
  currIndDelta_ = 0;
  prevInd_ = currInd_;
  targetFrame_ = targetFrame;
  closestWaypoint_ =
      path_utils::safeTransformPoses(*tfBuffer_, targetFrame_, {globalPath_->poses[currInd_]})[0];
}

bool PathDataExtd::configureProperties() {
  bool ret = true;
  for (size_t n = 0; n < properties_.size(); n++) {
    try {
      properties_[n]->configure(node_, propertyNames_[n], tfBuffer_, static_cast<PathData*>(this));
      RCLCPP_INFO(node_->get_logger(), "CONFIGURED -- Path property %s/%s", name_.c_str(),
                  propertyNames_[n].c_str());
    } catch (std::exception& ex) {
      RCLCPP_ERROR(node_->get_logger(),
                   "FAILED -- Path property %s/%s in configureProperties() Reason: %s",
                   name_.c_str(), propertyNames_[n].c_str(), ex.what());
      ret = false;
    }
  }
  return ret;
}

void PathDataExtd::setCurrentIndex(const int& newCurrentIndex, PathCandidate* globalPath) {
  if (globalPath != nullptr) globalPath_ = globalPath;
  prevInd_ = currInd_;
  currIndDelta_ = newCurrentIndex - currInd_;
  const int half = globalPath_->size() / 2;
  // Extreme case handling -- non of them should actually happen.
  if (currIndDelta_ < -half) {
    if (!isClosedPath()) {
      throw std::logic_error("LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
    } else {
      currIndDelta_ += globalPath_->size();
    }
  } else if (currIndDelta_ > half) {
    // We are heading the WRONG direction
    if (!isClosedPath()) {
      throw std::logic_error("LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
    } else {
      currIndDelta_ -= globalPath_->size();
    }
  }
  currInd_ = newCurrentIndex;
  closestWaypoint_ =
      path_utils::safeTransformPoses(*tfBuffer_, targetFrame_, {globalPath_->poses[currInd_]})[0];
}

bool PathDataExtd::setBonds(const std::vector<const PathData*>& bonds,
                            const std::vector<uint8_t>& prio, const bool& reset) {
  if (prio.size() != bonds.size()) {
    // input error, do nothing.
    RCLCPP_ERROR(node_->get_logger(),
                 "Vector of path data pointer size differs from priority vector size.");
    return false;
  }
  if (reset) bindings_.clear();
  bindings_.reserve(bindings_.size() + bonds.size());
  uint8_t max_prio = 0;
  uint8_t my_prio = 0;
  size_t leader = 0;
  for (size_t n = 0; n < bonds.size(); n++) {
    // record my priority, and compare which binding to sync with (the highest prio in the
    // list, or self, whichever higher)
    if (bonds[n] == this) {
      my_prio = prio[n];
      continue;
    }
    // make them const pointers so we don't mess up our peers.
    bindings_.emplace_back(bonds[n], prio[n]);
    if (prio[n] > max_prio) {
      max_prio = prio[n];
      leader = n;
    }
  }
  // perform index syncing of globalPath_
  if (max_prio > my_prio) {
    leaderPath_ = bonds[leader];
    const PathCandidate* leaderPath = leaderPath_->getGlobalPath();
    if (leaderPath == nullptr) {
      // leader not initialized yet. Fail here. revert changes.
      RCLCPP_ERROR(node_->get_logger(), "Leader path is a nullptr. Reverting changes.");
      bindings_.resize(bindings_.size() - bonds.size());
      return false;
    }
    std::vector<PoseStamped> syncedPoses;
    syncedPoses.reserve(leaderPath->size());
    size_t n = 0;
    // initially, perform a global search to align the start of the leaderPath.
    int ourInd =
        path_utils::findClosestInd(node_->get_logger(), *tfBuffer_, globalPath_->poses,
                                   leaderPath->poses[n], 0, false, globalPath_->isClosedPath);
    int frontInd = ourInd;
    int rearInd = ourInd;
    Pose closestSelfPointFront, closestSelfPointRear;
    while (true) {
      // loops through all leader path poses
      while (globalPath_->isClosedPath || rearInd > 0) {
        closestSelfPointRear = path_utils::safeProjection(*tfBuffer_, leaderPath->poses[n],
                                                          globalPath_->poses[rearInd]);
        if (closestSelfPointRear.position.x <= 0.) break;
        rearInd = (rearInd + globalPath_->size() - 1) % globalPath_->size();
      }
      while (globalPath_->isClosedPath || frontInd < static_cast<int>(globalPath_->size()) - 1) {
        closestSelfPointFront = path_utils::safeProjection(*tfBuffer_, leaderPath->poses[n],
                                                           globalPath_->poses[frontInd]);
        if (closestSelfPointFront.position.x >= 0.) break;
        frontInd = (frontInd + 1) % globalPath_->size();
      }
      double deltaX = closestSelfPointFront.position.x - closestSelfPointRear.position.x;
      if (deltaX == 0.)
        syncedPoses.emplace_back(globalPath_->poses[ourInd]);  // spot on
      else {
        // a linear interpolation is needed
        PoseStamped p = globalPath_->poses[rearInd];
        double ratio = -p.pose.position.x / deltaX;
        PoseStamped* p_front = &(globalPath_->poses[frontInd]);
        p.pose.position.x += ratio * (p_front->pose.position.x - p.pose.position.x);
        p.pose.position.y += ratio * (p_front->pose.position.y - p.pose.position.y);
        p.pose.position.z += ratio * (p_front->pose.position.z - p.pose.position.z);
        syncedPoses.emplace_back(p);
      }
      // prepare for processing the next point
      n++;
      if (n >= leaderPath->size()) break;
      // since we have the starting point, now we only perfrom localized searches.
      ourInd = path_utils::findClosestInd(node_->get_logger(), *tfBuffer_, globalPath_->poses,
                                          leaderPath->poses[n], rearInd, true,
                                          globalPath_->isClosedPath);
      frontInd = ourInd;
      rearInd = ourInd;
    }
    globalPath_->poses = syncedPoses;
    globalPath_->version = static_cast<unsigned long>(node_->now().nanoseconds());
  } else {
    leaderPath_ = this;
  }
  return true;
}

}  // namespace planning
