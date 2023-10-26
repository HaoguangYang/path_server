/**
 * @file path_server.cpp
 * @author Haoguang Yang (yang1510@purdue.edu)
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

#include "path_server/path_server.hpp"

#include <chrono>
#include <ctime>

namespace planning {

PathServer::PathServer(NodePtr node, tf2_ros::Buffer* tfBuffer, const std::string& name,
                       const std::string& targetFrame, const std::string& utmFrame,
                       const std::string& localEnuFrame, const std::string& mapFrame,
                       const std::string& odomFrame, const std::string& onNewPath,
                       const bool& doInterp, const double& interpStep, const bool& interpretYaw,
                       const std::vector<std::pair<std::string, std::string>>& plugins,
                       const double& utmScale)
    : node_(node),
      tfBuffer_(tfBuffer),
      pathFrame_("earth"),
      targetFrame_(targetFrame),
      onNewPath_(onNewPath),
      doInterp_(doInterp),
      interpretYaw_(interpretYaw),
      interpStep_(interpStep),
      // assign header frame ids for path representations
      utmCoords_(utmFrame),
      localEnuCoords_(localEnuFrame),
      mapCoords_(mapFrame),
      odomCoords_(odomFrame),
      utmScale_(utmScale),
      pathData_{node, tfBuffer, name, plugins} {
  pathData_.setTargetFrame(targetFrame);
}

template <typename T, typename Lambda1, typename Lambda2>
std::vector<PoseStamped> PathServer::packPathPoses(const std::vector<T>& coords,
                                                   const std::string& outputFrame,
                                                   const bool& isClosedPath, Lambda1&& poseMethod,
                                                   Lambda2&& headingMethod) {
  std::vector<PoseStamped> out;
  std::vector<double> yawOut;
  out.reserve(coords.size());
  yawOut.reserve(coords.size());
  for (const auto& coord : coords) {
    T transformedPose = poseMethod(coord);
    out.emplace_back([&] {
      PoseStamped outPose;
      outPose.header.frame_id = outputFrame;
      outPose.pose.position.x = transformedPose[0];
      outPose.pose.position.y = transformedPose[1];
      if (transformedPose.size() > 2)
        outPose.pose.position.z = transformedPose[2];
      else
        outPose.pose.position.z = 0.0;
      return outPose;
    }());
    if (coord.size() > 2) yawOut.emplace_back(headingMethod(coord[2]));
    // std::cout << outPose.pose.position.x - out[0].pose.position.x << " " <<
    // outPose.pose.position.y - out[0].pose.position.y << std::endl;
  }
  /* Do a global interpolation of path
   * If global interp, the advancement of current/LA/LB indices per timestep is 1, which means the
   * update step performs no more than two trials. This is a tolerable overhead compared to
   * calculating the local interpolation every step.
   */
  if (doInterp_) {
    // organize out[i].pose in an Eigen matrix
    MatrixXd positions = MatrixXd(coords.size(), 3);
    int i = 0;
    for (const auto& pose : out) {
      positions.row(i) << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
      i++;
    }
    // std::cout << positions - VectorXd::Ones(positions.rows())*positions.row(1) << std::endl;

    // initialize bezier curve fitting tool
    auto interp = BezierCurveInterpolator(positions, isClosedPath);
    interp.setInterpDistance(interpStep_);
    MatrixXd interp_out = interp.evaluate();
    // std::cout << interp_out - VectorXd::Ones(interp_out.rows())*interp_out.row(0) << std::endl;

    // re-populate output path vector
    out.clear();
    out.reserve(interp_out.rows());
    // for (int n = interpOffset; n < interpCutoff; n++){
    for (int n = 0; n < interp_out.rows(); n++) {
      out.emplace_back([&] {
        PoseStamped pose;
        pose.header.frame_id = outputFrame;
        pose.pose.position.x = interp_out(n, 0);
        pose.pose.position.y = interp_out(n, 1);
        pose.pose.position.z = 0.0;
        return pose;
      }());
    }

    if (coords[0].size() > 2) {
      double* ptr = &yawOut[0];
      Map<VectorXd> angles(ptr, yawOut.size());
      // post-processing: no difference should exceed M_PI
      const double TWO_PI = M_PIf64 + M_PIf64;
      for (int n = 1; n < angles.size(); n++) {
        if (angles(n) - angles(n - 1) > M_PIf64)
          angles(n) -= TWO_PI;
        else if (angles(n) - angles(n - 1) < -M_PIf64)
          angles(n) += TWO_PI;
      }

      VectorXd angInterp = interp.evaluateAnother(angles);

      yawOut.clear();
      yawOut.reserve(interp_out.rows());
      // for (int n = interpOffset; n < interpCutoff; n++){
      for (int n = 0; n < interp_out.rows(); n++) {
        yawOut.emplace_back(angInterp(n));
      }
    }
  }
  if (coords[0].size() > 2) {
    std::transform(yawOut.cbegin(), yawOut.cend(), out.cbegin(), out.begin(),
                   [&](const double& yaw, const PoseStamped& pose) {
                     tf2::Quaternion q;
                     q.setRPY(0.0, 0.0, yaw);
                     PoseStamped ret = pose;
                     ret.pose.orientation = tf2::toMsg(q);
                     return ret;
                   });
  } else if (!interpretYaw_) {
    std::for_each(out.begin(), out.end(), [](PoseStamped& p) {
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;
      p.pose.orientation.z = 0.0;
      p.pose.orientation.w = 1.0;
    });
  } else {
    // interpret yaw from neighboring points
    for (auto it = out.begin(); true; ++it) {
      if (it == (out.end() - 1)) {
        if (!isClosedPath) {
          // open path, last element does not turn
          it->pose.orientation = (it - 1)->pose.orientation;
        } else {
          // close the loop
          tf2::Quaternion q = path_utils::safeGetOrientation(*tfBuffer_, *(it), out.front());
          it->pose.orientation = tf2::toMsg(q);
        }
        break;
      }
      // Use atan2 to obtain yaw angle, then express in quaternion
      tf2::Quaternion q = path_utils::safeGetOrientation(*tfBuffer_, *(it), *(it + 1));
      it->pose.orientation = tf2::toMsg(q);
    }
  }
  return out;
}

void PathServer::registerPath(const std::string& pathFrame,
                              const std::vector<std::vector<double>>& rawPath,
                              const bool& isClosedPath, const std::string& pathName) {
  PathCandidate* which = &utmCoords_;
  std::vector<PoseStamped> regPath;
  // isolate this variable because it requires re-initialization.
  if (pathFrame == "earth") {
    // First to UTM
    regPath = packPathPoses(
        rawPath, utmCoords_.frame_id, isClosedPath,
        // Lat/Long to UTM lambda function
        [&](const std::vector<double>& latlon) {
          double utmE, utmN;
          int utmZoneId;
          bool northp_tmp;
          double gamma_tmp;
          double utmScale_tmp;
          GeographicLib::UTMUPS::Forward(latlon[0], latlon[1], utmZoneId, northp_tmp, utmE, utmN,
                                         gamma_tmp, utmScale_tmp);
          // std::cout << utmE << " " << utmN << std::endl;
          return std::vector<double>{utmE, utmN};
        },
        // NED heading to ENU yaw lambda function
        [](const double& heading) { return (90.0 - heading) * M_PI / 180.0; });
  } else {
    try {
      which = path_utils::findFirstThat(
          {&utmCoords_, &mapCoords_, &localEnuCoords_},
          [](const auto& item, const std::string& toFind_) { return item->frame_id == toFind_; },
          pathFrame);
      // directly populate utm, map, or local_enu coords
      regPath = packPathPoses(rawPath, pathFrame, isClosedPath);
    } catch (std::runtime_error& e) {
      // Transform here if the path frame is non of the static frames.
      std::vector<PoseStamped> tmp_path = packPathPoses(rawPath, pathFrame, isClosedPath);
      // retries for the initial 10 seconds
      uint8_t retries = 100;
      while (
#ifdef RCLCPP__RCLCPP_HPP_
          rclcpp::ok()
#elif defined ROSCPP_ROS_H
          node_->ok()
#endif
      ) {
        // most likely this path is w.r.t. some perceived stationary object.
        // tries in the precedence order of map -> local_enu -> utm -> odom
        try {
          which = path_utils::findFirstThat(
              {&mapCoords_, &localEnuCoords_, &utmCoords_, &odomCoords_},
              [this](const auto& item, const std::string& toFrame_) {
                return path_utils::safeCanTransform(*(this->tfBuffer_), toFrame_, item->frame_id,
                                                    tf2::TimePointZero);
              },
              pathFrame);
        } catch (std::runtime_error& e) {
          // we got nothing this round. wait 0.1s and try again
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
                               "No transform was available. Retrying...");
#ifdef RCLCPP__RCLCPP_HPP_
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
#elif defined ROSCPP_ROS_H
          ros::Duration(0.1).sleep();
#endif
          retries--;
          if (retries > 0) continue;
          RCLCPP_FATAL(
              node_->get_logger(),
              "PROVIDED PATH FRAME IS NOT TRANSFORM-ABLE. THE NODE WILL NOT OUTPUT ANYTHING.");
          throw std::runtime_error("Path Registration Failed.");
        }
        regPath = path_utils::safeTransformPoses(*tfBuffer_, which->frame_id, tmp_path);
        // we got a valid output.
        if (regPath.size()) break;
      }
    }
  }
  // initial path or modifies path. This step includes a forced (re-)initialization.
  if (!which->poses.size()) {
    which->poses = regPath;
    // initialization of frenet frame and register path
    while (
#ifdef RCLCPP__RCLCPP_HPP_
        rclcpp::ok()
#elif defined ROSCPP_ROS_H
        node_->ok()
#endif
    ) {
      if (initialize(true)) break;
#ifdef RCLCPP__RCLCPP_HPP_
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
#elif defined ROSCPP_ROS_H
      ros::Duration(0.1).sleep();
#endif
    }
  } else if (onNewPath_ == "append")
    appendPoses(which->poses, regPath);
  else if (onNewPath_ == "replace")
    replacePoses(which->poses, regPath);
  // else if (onNewPath_ == "intersect")
  //   intersectPoses(which.poses, regPath);
  //  TODO: more path insertion approaches
  //  give it a new time stamp
  // which.header.stamp = node_->now();
  which->version =
#ifdef RCLCPP__RCLCPP_HPP_
      static_cast<unsigned long>(node_->now().nanoseconds());
#elif defined ROSCPP_ROS_H
      ros::Time::now().toNSec();
#endif
  which->isClosedPath = isClosedPath;
  latestPathVer_ = which->version;
  pathName_ = pathName;
  staticFrameChkTimer_ =
#ifdef RCLCPP__RCLCPP_HPP_
      node_->create_wall_timer(1.0s, std::bind(&PathServer::updateStaticFrameTfAvail, this));
#elif defined ROSCPP_ROS_H
      node_->createTimer(ros::Duration(1.0), &PathServer::updateStaticFrameTfAvail, this);
#endif
}

bool PathServer::initialize(const bool& force) {
  // path has already been initialized.
  if (pathData_.getCurrentIndex() >= 0 && !force) return true;

  // try each representation of the path until the any one of them transforms to the target pose.
  // find the first one that is both available and transformable.
  PathCandidate* which = &utmCoords_;
  try {
    which = path_utils::findFirstThat(
        {&utmCoords_, &localEnuCoords_, &mapCoords_, &odomCoords_},
        [this](const auto& item, const std::string& toFrame) {
          // skip if this path candidate is not the latest or is empty
          if (item->version < latestPathVer_ || item->poses.size() <= 0) return false;
          // otherwise, use the one that can be transformed to the target frame
          return path_utils::safeCanTransform(*(this->tfBuffer_), toFrame, item->frame_id,
                                              tf2::TimePointZero);
        },
        targetFrame_);
  } catch (std::runtime_error& e) {
    // none of them can. return uninitialized.
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 1000U,
        "INITIALIZATION FAILED due to unable to transform poses to %s. Retrying...",
        targetFrame_.c_str());
    return false;
  }
  std::vector<PoseStamped> transformedPoses =
      path_utils::safeTransformPoses(*tfBuffer_, targetFrame_, which->poses);

  // initializing
  pathData_.configure(findClosestInd(transformedPoses, 0, false), which, targetFrame_);

  // return initialization status
  return (pathData_.getCurrentIndex() >= 0);
}

bool PathServer::configureProperties() {
  // initializing
  return pathData_.configureProperties();
}

void PathServer::updateStaticFrameTfAvail(
#ifdef ROSCPP_ROS_H
    const ros::TimerEvent&
#endif
) {
  // if has a static frame but its path representation does not exist, create one.
  // from local ENU or map to UTM
  syncPathCandidate({localEnuCoords_, mapCoords_}, utmCoords_);
  // from UTM or local ENU to Map
  syncPathCandidate({utmCoords_, localEnuCoords_}, mapCoords_);

  // odom frame coordinates will NOT propagate outwards, but will ONLY propages
  // inwards (convert from X to odom frame), since it is NOT a strictly static frame.
  // always update odom coords copy because odom frame can drift significantly.
  // FIXME: this is not the best resolution -- obtain the tf from whatever frame to odom frame,
  // and express self pose in odom frame to obtain the distance from self to path in fixed frame.
  odomCoords_.version = 0;
  syncPathCandidate({utmCoords_, localEnuCoords_, mapCoords_}, odomCoords_);
}

/* publish path in target_frame, extracting the segment within
 * [lookBehind, lookAhead]
 * */

bool PathServer::updateIndex() {
  // TODO: if path has not been initialized yet: clear and return empty path.
  if (pathData_.getCurrentIndex() < 0) return false;

  // std::vector<PoseStamped> *which;
  PathCandidate* which = &localEnuCoords_;
  try {
    // which =
    //     firstTransformable({odomCoords_, localEnuCoords_, utmCoords_, mapCoords_}, targetFrame_);
    which = path_utils::findFirstThat(
        {&utmCoords_, &localEnuCoords_, &mapCoords_, &odomCoords_},
        [this](PathCandidate* item, const std::string& toFrame_) {
          return path_utils::safeCanTransform(*(this->tfBuffer_), toFrame_, item->frame_id,
                                              tf2::TimePointZero) &&
                 item->poses.size();
        },
        targetFrame_);
  } catch (std::runtime_error& e) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
                          "Cannot transform frames or the poses are not initialized.");
    // TODO: cannot transform: clear and return empty path.
    return false;
  }
  // start from the current index, and determine when to increment the current index.
  if (!pathData_.isLeader()) {
    RCLCPP_DEBUG(node_->get_logger(), "UPDATING A NON-LEADER PATH");
    pathData_.setCurrentIndex(pathData_.getLeader()->getCurrentIndex(), which);
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "UPDATING A LEADER PATH");
    int newClosestInd =
        findClosestInd(which->poses, pathData_.getCurrentIndex(), true, which->isClosedPath);
    if (newClosestInd < 0) {
      return false;
    }
    pathData_.setCurrentIndex(newClosestInd, which);
  }

  return true;
}

void PathServer::syncPathCandidate(const std::initializer_list<PathCandidate>& in,
                                   PathCandidate& out) {
  if (out.version >= latestPathVer_) {
    // RCLCPP_DEBUG(node_->get_logger(), "No need to synchronize %s.", out.frame_id.c_str());
    return;
  }
  if (!path_utils::safeCanTransform(*tfBuffer_, out.frame_id, targetFrame_, tf2::TimePointZero)) {
    // RCLCPP_DEBUG(node_->get_logger(), "Cannot transform from %s to %s",
    //             out.frame_id.c_str(), targetFrame_.c_str());
    return;
  }
  std::vector<PoseStamped> tmpPoses;
  std::string adoptedFrame;
  bool isClosedPathTmp;
  for (const auto& it : in) {
    if (it.version != latestPathVer_) continue;
    tmpPoses = path_utils::safeTransformPoses(*tfBuffer_, out.frame_id, it.poses);
    // Handle UTM -> local cartesian scaling (last bit of at most 0.1% error)
    if (it.frame_id == utmCoords_.frame_id && it.frame_id != out.frame_id) {
      for (auto& p : tmpPoses) {
        p.pose.position.x /= utmScale_;
        p.pose.position.y /= utmScale_;
      }
    }
    isClosedPathTmp = it.isClosedPath;
    adoptedFrame = it.frame_id;
    if (tmpPoses.size()) {
      // got a valid transform
      out.poses = tmpPoses;
      out.version = latestPathVer_;
      out.isClosedPath = isClosedPathTmp;
      RCLCPP_DEBUG(node_->get_logger(), "Synchronized path representation %s from %s.",
                   out.frame_id.c_str(), adoptedFrame.c_str());
      break;
    }
  }
  // if (!tmpPoses.size()) {
  //   RCLCPP_DEBUG(node_->get_logger(), "Failed to synchronize path representation %s from %s.",
  //               out.frame_id.c_str(), adoptedFrame.c_str());
  // }
}

int PathServer::findClosestInd(const std::vector<PoseStamped>& poses, const int& startInd,
                               bool stopAtFirstMin, bool wrapAround) const {
  const PoseStamped self = [&]() {
    PoseStamped ret;
    // ret.header.stamp = node_->now();  // you shouldn't do this.
    ret.header.frame_id = targetFrame_;
    ret.pose.position.x = 0.0;
    ret.pose.position.y = 0.0;
    ret.pose.position.z = 0.0;
    ret.pose.orientation.x = 0.0;
    ret.pose.orientation.y = 0.0;
    ret.pose.orientation.z = 0.0;
    ret.pose.orientation.w = 1.0;
    return ret;
  }();
  return path_utils::findClosestInd(node_, *tfBuffer_, poses, self, startInd, stopAtFirstMin,
                                    wrapAround);
}

/*
  if (!prepareActivePathSeg_) {
    currentInd_ = newClosestInd;
    // do not create a full path, but only generate the closest point on the path.
#ifdef RCLCPP__RCLCPP_HPP_
    frontPath_.header.stamp = node_->now();
#elif defined ROSCPP_ROS_H
    frontPath_.header.stamp = ros::Time::now();
#endif
    frontPath_.header.frame_id = targetFrame_;
    frontPath_.poses =
        safeTransformPoses(targetFrame_, std::vector<PoseStamped>{which->poses[currentInd_]});
    frontPath_.look_ahead = {static_cast<float>(frontPath_.poses[0].pose.position.x)};
    frontPath_.offset_left = {0.};
    frontPath_.curvature = {curvature_[currentInd_]};
    frontPath_.offset_poses = frontPath_.poses;
    rearPath_ = frontPath_;
    return true;
  }

  int delta = newClosestInd - currentInd_;
  const int half = which->poses.size() / 2;
  if (delta < -half) {
    if (!isClosedPath_) {
#ifdef RCLCPP_FATAL
      RCLCPP_FATAL(node_->get_logger(), "LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
#elif defined ROS_FATAL
      ROS_FATAL("LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
#endif
    } else {
      lapCount_++;
      delta += which->poses.size();
    }
  } else if (delta > half) {
    // We are heading the WRONG direction
#ifdef RCLCPP_WARN
    RCLCPP_WARN(node_->get_logger(), "WE ARE HEADING TOWARDS THE WRONG DIRECTION!");
#elif defined ROS_WARN
    ROS_WARN("WE ARE HEADING TOWARDS THE WRONG DIRECTION!");
#endif
    if (!isClosedPath_) {
#ifdef RCLCPP_FATAL
      RCLCPP_FATAL(node_->get_logger(), "LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
#elif defined ROS_FATAL
      ROS_FATAL("LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
#endif
    } else {
      lapCount_--;
      delta -= which->poses.size();
    }
  }

  // update odometer
  odometer_ += getFrenetXDistance(currentInd_, currentInd_ + delta);
  currentInd_ = newClosestInd;

  // Now, generate the pose list from currentInd_, extending forward and backward.
  updateLookAheadAndBehindInds();

  // Transform-sensitive fields
  // transform poses. This needs to be done every cycle.
#ifdef RCLCPP__RCLCPP_HPP_
  frontPath_.header.stamp = node_->now();
#elif defined ROSCPP_ROS_H
  frontPath_.header.stamp = ros::Time::now();
#endif
  frontPath_.header.frame_id = targetFrame_;
  // update active path segments from stored path representations.
  frontPath_.poses = safeTransformPoses(
      targetFrame_, getActiveSeg(which->poses, currentInd_, lookAheadInd_, isClosedPath_));
#ifdef RCLCPP__RCLCPP_HPP_
  rearPath_.header.stamp = node_->now();
#elif defined ROSCPP_ROS_H
  rearPath_.header.stamp = ros::Time::now();
#endif
  rearPath_.header.frame_id = targetFrame_;
  rearPath_.poses = safeTransformPoses(
      targetFrame_, getActiveSeg(which->poses, currentInd_, lookBehindInd_, isClosedPath_));
*/

/* PeerHandle Class Methods */

PeerHandle::PeerHandle(const std::vector<PathServer*>& peers, const std::vector<uint8_t>& prio) {
  if (peers.size() != prio.size()) {
    peers_.clear();
  } else {
    for (size_t i = 0; i < peers.size(); ++i) {
      peers_.emplace_back(peers[i], prio[i]);
      maxNumPropertiesPerPath_ = std::max(maxNumPropertiesPerPath_, peers[i]->getNumProperties());
    }
    std::sort(peers_.begin(), peers_.end(),
              [](const std::pair<PathServer*, uint8_t>& lhs,
                 const std::pair<PathServer*, uint8_t>& rhs) { return lhs.second > rhs.second; });
  }
}

PeerHandle::PeerHandle(const std::vector<PathServer*>& peers) {
  for (size_t i = 0; i < peers.size(); ++i) {
    peers_.emplace_back(peers[i], 0);
    maxNumPropertiesPerPath_ = std::max(maxNumPropertiesPerPath_, peers[i]->getNumProperties());
  }
}

bool PeerHandle::configureProperties(const bool& fastFail) {
  bool ret = true;
  for (auto& peer : peers_) {
    // std::cout << "Configuring property for:" << peer.first->getPathData().getName() << std::endl;
    ret &= peer.first->configureProperties();
    if (fastFail && !ret) return false;
  }
  return ret;
}

bool PeerHandle::update() {
  // First update index for all
  for (auto& peer : peers_) {
    if (!peer.first->updateIndex()) return false;
  }
  // Then update properties in an interleaving pattern
  for (size_t propertyIdx = 0; propertyIdx < maxNumPropertiesPerPath_; propertyIdx++) {
    for (size_t peerIdx = 0; peerIdx < peers_.size(); peerIdx++) {
      if (propertyIdx >= peers_[peerIdx].first->getNumProperties()) continue;
      if (!peers_[peerIdx].first->updateProperty(propertyIdx)) return false;
    }
  }
  return true;
}

}  // namespace planning
