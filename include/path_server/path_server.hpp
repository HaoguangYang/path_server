/**
 * @file path_server.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief The path server class. It manages multi-reference frame switching as a redundancy, path
 * registration, and periodic updates.
 * @version 0.1
 * @date 2023-08-12
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

#ifndef _PATH_SERVER__PATH_SERVER_HPP_
#define _PATH_SERVER__PATH_SERVER_HPP_

#include <chrono>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <stdexcept>

#include "GeographicLib/UTMUPS.hpp"

// ROS library and messages
#if __has_include("rclcpp/rclcpp.hpp")
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#elif __has_include("ros/ros.h")
#include "geometry_msgs/PoseStamped.h"
#include "path_server/logging.hpp"
#include "ros/ros.h"
#endif

// In-package functional components
#include "bezier_interpolation.hpp"
#include "path_data_extd.hpp"
#include "path_property.hpp"
#include "path_utils.hpp"

// TF2 utilities
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace Eigen;

namespace planning {

#ifdef RCLCPP__RCLCPP_HPP_
using geometry_msgs::msg::PoseStamped;
#elif defined ROSCPP_ROS_H
using geometry_msgs::PoseStamped;
// TimePointZero already defined in path_utils.hpp
#endif

// The abstract class as a place-holder.
class PathServer;

/**
 * @brief A class that contains multiple path server instances that are contextually related. Their
 * property configuration and update process are orchistrated.
 */
class PeerHandle {
  /**
   * @brief The vector and priority of the path server instances that are related.
   */
  std::vector<std::pair<PathServer*, uint8_t>> peers_ = {};

  /**
   * @brief max number of properties within one path server instance. Used to update properties in
   * an interleaved pattern.
   */
  size_t maxNumPropertiesPerPath_ = 0;

 public:
  /**
   * @brief Construct a new Peer Handle object, with included peers being called in the order of
   * their priorities.
   *
   * @param peers input vector of pointers to path server instances.
   * @param prio input vector of the priority of the path server instances. Instances are sorted by
   * their priorities before storing into the handle. When configuring properties and updating, the
   * instances are called with descreasing order in their priorities.
   */
  PeerHandle(const std::vector<PathServer*>& peers, const std::vector<uint8_t>& prio);

  /**
   * @brief Construct a new Peer Handle object, with included peers being called in the order of
   * inclusion. All included instances have the same priority value of 0.
   *
   * @param peers input vector of pointers to path server instances.
   */
  PeerHandle(const std::vector<PathServer*>& peers);

  /**
   * @brief Configure properties associated with individual path server instances. The instances
   * with higher priorities will be configured first.
   *
   * @param fastFail Whether to fail and exit at the first occurance of configuration failure.
   * @return true The configuration of all instances is successful.
   * @return false At least one instance have failed in the configuration process.
   */
  bool configureProperties(const bool& fastFail = true);

  /**
   * @brief Update all included path server instances at the decreasing order of their priorities
   * (or inclusion order if they have the same priority).
   *
   * @return true All instances updated successfully.
   * @return false At least one instance have failed to update.
   */
  bool update();

  /**
   * @brief Number of contained peers.
   *
   * @return const size_t& size of the peers_ vector.
   */
  size_t size() const { return peers_.size(); }

  /**
   * @brief Indexing method of contained peers
   *
   * @param i which peer to index
   * @return const PathServer* read-only pointer to the contained PathServer instance.
   */
  const PathServer* at(const size_t& i) const { return peers_[i].first; }
};

/**
 * @brief The actual definition of the path server class. A path server manages one instance of path
 * data, which is initialized from raw coordinates (from a file). Multiple representations of the
 * path data in different reference frames are generated to mitigate unplanned loss of tf updates,
 * change of tf configurations, etc. During update, the path server performs fault-tolerant
 * selection of the closest coordinate in the given path, from a list of candidate frame ids. The
 * path server then links the result to the managed path data instance accordingly. Finally, the
 * associated properties of the managed path data will be updated based on configuration order.
 */
class PathServer {
 public:
#ifdef RCLCPP__RCLCPP_HPP_
  typedef rclcpp::Node* NodePtr;
#elif defined ROSCPP_ROS_H
  typedef ros::NodeHandle* NodePtr;
#endif

  explicit PathServer(NodePtr node, tf2_ros::Buffer* tfBuffer, const std::string& name,
                      const std::string& targetFrame, const std::string& utmFrame,
                      const std::string& localEnuFrame, const std::string& mapFrame,
                      const std::string& odomFrame, const std::string& onNewPath,
                      const bool& doInterp, const double& interpStep, const bool& interpretYaw,
                      const std::vector<std::pair<std::string, std::string>>& plugins,
                      const double& utmScale = 1.0);

  ~PathServer() = default;

  /**
   * @brief convert x_file_, y_file_, yaw_file_ to a static standard frame
   * (map/utm/local_enu). This step does not require transform tree.
   *
   * @param pathFrame
   * @param rawPath
   */
  void registerPath(const std::string& pathFrame, const std::vector<std::vector<double>>& rawPath,
                    const bool& isClosedPath, const std::string& pathName = "");

  /**
   * @brief find current closest point on the path coordinates,
   * and output the index as a class variable.
   *
   * @return true
   * @return false
   */
  bool initialize(const bool& force = false);

  bool configureProperties();

  /**
   * @brief publish path in target_frame, extracting the segment within [lookBehind, lookAhead]
   *
   * @return true
   * @return false
   */
  bool updateIndex();

  bool updateProperty(const size_t& which) { return pathData_.updateProperty(which); }

  /**
   * @brief Sync paths in different static reference frames. The availability of these frames may
   * change over time.
   */
#ifdef RCLCPP__RCLCPP_HPP_
  void updateStaticFrameTfAvail();
#elif defined ROSCPP_ROS_H
  void updateStaticFrameTfAvail(const ros::TimerEvent&);
#endif

  const int& getCurrentInd() const { return pathData_.getCurrentIndex(); }

  const bool& isClosedPath() const { return pathData_.isClosedPath(); }

  const PoseStamped& getClosestWaypoint() const { return pathData_.getClosestWaypoint(); }

  const PathData& getPathData() const { return pathData_; }

  size_t getNumProperties() const { return pathData_.getNumProperties(); }

  void setPeers(const std::vector<const PathData*>& peers) { pathData_.setPeers(peers); }

  /**
   * @brief Set the Peers object: a cluster of paths with the same context and can be updated
   * together, regardless of order.
   *
   * @param peers the pointers to PathServer instances in which the paths of interest are wrapped.
   * @return PeerHandle periodic update handle.
   */
  static PeerHandle setPeers(const std::vector<PathServer*>& peers) {
    std::vector<const PathData*> pdPeers(peers.size());
    for (const auto& ps : peers) {
      pdPeers.emplace_back(&(ps->getPathData()));
    }
    for (auto& ps : peers) {
      ps->setPeers(pdPeers);
    }
    return PeerHandle(peers);
  }

  bool setBonds(const std::vector<const PathData*>& peers, const std::vector<uint8_t>& prio) {
    return pathData_.setBonds(peers, prio);
  }

  /**
   * @brief Set the Peers object with priority order, i.e. a Bonds object. The bond is a cluster of
   * paths with the same context and synchronized contents, and can be updated together with a
   * leader-follower fashion. The leader in the bond has higher priority and is updated first.
   *
   * @param peers the pointers to PathServer instances in which the paths of interest are wrapped.
   * @param prio the vector of uint8_t describing update priority
   * @return PeerHandle periodic update handle.
   */
  static PeerHandle setBonds(const std::vector<PathServer*>& peers,
                             const std::vector<uint8_t>& prio) {
    std::vector<const PathData*> pdPeers{};
    pdPeers.reserve(peers.size());
    for (const auto& ps : peers) {
      pdPeers.emplace_back(&(ps->getPathData()));
    }
    bool success = true;
    for (auto& ps : peers) {
      success &= ps->setBonds(pdPeers, prio);
    }
    if (!success) return PeerHandle({});
    return PeerHandle(peers, prio);
  }

  std::vector<bool> getPluginActivationStates() const { return pathData_.getActivationStates(); }

  void setPluginActivationStates(const std::vector<bool>& activation) {
    RCLCPP_INFO(node_->get_logger(), "Setting plugin activation status for path: %s",
                pathName_.c_str());
    pathData_.setActivationStates(activation);
  }

 private:
  /**
   * @brief directly pack raw vector of poses (from file) into path.poses. It uses a passthrough
   * method for position and orientation.
   *
   * @tparam T input vector data type
   * @param coords input vector of raw poses without frame information
   * @param outputFrame desired frame id of the output
   * @return std::vector<PoseStamped> output vector of poses that can be directly assigned to
   * nav_msgs::msg::Path.poses
   */
  template <typename T>
  std::vector<PoseStamped> packPathPoses(const std::vector<T>& coords,
                                         const std::string& outputFrame, const bool& isClosedPath) {
    return packPathPoses(
        coords, outputFrame, isClosedPath, [](const T& xp) { return xp; },
        [](const auto& xr) { return xr; });
  }

  /**
   * @brief pack raw vector of poses (from file) into path.poses with
   * custom transformation functions
   *
   * @tparam T input vector data type
   * @tparam Lambda1 function type for position
   * @tparam Lambda2 function type for orientation
   * @param coords raw vector of coordinates
   * @param outputFrame desired frame id of the output
   * @param poseMethod position function handle, such that `T transformedPose =
   * poseMethod(coords[n]);`
   * @param headingMethod orientation function handle, such that if the third field is valid,
   * `double heading = headingMethod(coords[n][2]);`
   * @return std::vector<PoseStamped> output vector of poses that can be directly assigned to
   * nav_msgs::msg::Path.poses
   */
  template <typename T, typename Lambda1, typename Lambda2>
  std::vector<PoseStamped> packPathPoses(const std::vector<T>& coords,
                                         const std::string& outputFrame, const bool& isClosedPath,
                                         Lambda1&& poseMethod, Lambda2&& headingMethod);

  /**
   * @brief A path registration method that appends new poses `input` at the end of existing poses
   * `target`.
   *
   * @param target existing poses
   * @param input new poses to be appended
   */
  void appendPoses(std::vector<PoseStamped>& target, const std::vector<PoseStamped>& input);

  /**
   * @brief A path registration method that replaces existing poses `target` with new poses `input`.
   *
   * @param target existing poses
   * @param input new poses to replace the existing poses with.
   */
  void replacePoses(std::vector<PoseStamped>& target, const std::vector<PoseStamped>& input);

  // void intersectPoses(std::vector<PoseStamped> &target, const std::vector<PoseStamped> &input);

  void syncPathCandidate(const std::initializer_list<PathCandidate>& in, PathCandidate& out);

  /**
   * @brief This function finds the index on the path that is closest to the current
   * position. The returned index is in the range of [0, poses.size()]
   *
   * @param poses The vector of poses to which the distance from self is calculated
   * @param startInd  Starting from this index in the provided pose vector
   * @param stopAtFirstMin  Whether to stop searching when a first minima is found
   * @param wrapAround  Whether to re-start search from the beginning once reaching the end
   * @return int  The index of the closest pose.
   */
  int findClosestInd(const std::vector<PoseStamped>& poses, const int& startInd,
                     bool stopAtFirstMin = true, bool wrapAround = false) const;

  /*
    void syncPathCandidate(const std::initializer_list<PathCandidate>& in, PathCandidate& out) {
      if (out.version >= latestPathVer_) return;
      if (!safeCanTransform(out.frame_id, targetFrame_, tf2::TimePointZero)) return;
      std::vector<PoseStamped> tmpPoses;
      std::string adoptedFrame;
      for (const auto& it : in) {
        if (it.version == latestPathVer_) {
          tmpPoses = safeTransformPoses(out.frame_id, it.poses);
          // Handle UTM -> local cartesian scaling (last bit of at most 0.1% error)
          if (it.frame_id == utmCoords_.frame_id && it.frame_id != out.frame_id){
            for (auto& p : tmpPoses){
              p.pose.position.x /= utmScale_;
              p.pose.position.y /= utmScale_;
            }
          }
          adoptedFrame = it.frame_id;
        }
        if (!tmpPoses.size()) continue;
        break;
      }
      // got a valid transform
      if (tmpPoses.size()) {
        out.poses = tmpPoses;
        out.version = latestPathVer_;
  #ifdef RCLCPP_DEBUG
        RCLCPP_DEBUG(node_->get_logger(), "Synchronized path representation %s from %s.",
                     out.frame_id.c_str(), adoptedFrame.c_str());
  #elif defined ROS_DEBUG
        ROS_DEBUG("Synchronized path representation %s from %s.",
                     out.frame_id.c_str(), adoptedFrame.c_str());
  #endif
      }
    }
  */

  NodePtr node_;
  tf2_ros::Buffer* tfBuffer_;

  /**
   * @brief User-specified expression of the raw path and the targeted transformation.
   */
  std::string pathFrame_, targetFrame_;
  std::string pathName_ = "";

  std::string onNewPath_;
  bool doInterp_, interpretYaw_;
  double interpStep_;

  /**
   * @brief static or quasi-static reference frames that we will check.
   * Standard frame definitions:
   * earth: path file is in latitude, longitude, [optional - NED haeding (deg)] format.
   *        This is pre-transformed to utm before further transformation.
   * utm:   path file is in easting, northing, [optional - ENU heading (rad)] format.
   *        Standard tf2 transformation will be applied. UTM coordinates involve a scaling factor
   *        related to projection, and is either multiplied-in by the remainder of the stack or not.
   *        A parameter is set to control the application of the scaling factor.
   * local_enu: path file is in easting, northing, [optional - ENU heading (rad)] format.
   *        Standard tf2 transformation will be applied.
   * map:   path file is absolute x, y, [optional right-handed heading (rad)] format
   *        relative to the map frame. Standard tf2 transformation will be applied.
   * base_link: rigidly attached to the robot, facing front (X), left (Y), up (Z).
   */
  PathCandidate utmCoords_, localEnuCoords_, mapCoords_, odomCoords_;
  unsigned long latestPathVer_ = 0;
  double utmScale_ = 1.0;

#ifdef RCLCPP__RCLCPP_HPP_
  rclcpp::TimerBase::SharedPtr staticFrameChkTimer_{nullptr};
#elif defined ROSCPP_ROS_H
  ros::Timer staticFrameChkTimer_;
#endif

  // path in the selected reference frame and path property plugins
  PathDataExtd pathData_;

};  // class

}  // namespace planning

#endif  // _PATH_SERVER__PATH_SERVER_HPP_
