/**
 * @file lateral_offset.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief A plugin for path server that offsets a path locally and laterally, baed on the received
 * command. It uses linear interpolation to continuously move from the current path to the offseted
 * path. It also re-calculates the curve length and curvature that are altererd with the offset
 * path.
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

#ifndef PATH_PROPERTY_LATERAL_OFFSET_HPP
#define PATH_PROPERTY_LATERAL_OFFSET_HPP

#include <deque>
#include <rclcpp/rclcpp.hpp>

#include "path_server/path_data.hpp"
#include "path_server/path_properties/active_segment.hpp"
#include "path_server/path_properties/curvature.hpp"
#include "path_server/path_properties/curve_length.hpp"
#include "path_server/path_property.hpp"
#include "path_server/path_utils.hpp"

#ifdef RCLCPP__RCLCPP_HPP_
#include "path_server/msg/path_offset_command.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#elif defined ROSCPP_ROS_H
#include "path_server/PathOffsetCommand.h"
#include "std_msgs/Float32MultiArray.h"
#endif

namespace planning {

#ifdef RCLCPP__RCLCPP_HPP_
using path_server::msg::PathOffsetCommand;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Float32MultiArray;
#elif defined ROSCPP_ROS_H
using path_server::PathOffsetCommand;
using geometry_msgs::PoseStamped;
using std_msgs::Float32MultiArray;
#endif

using namespace path_utils;

class LateralOffset : public PathProperty {
 public:
  virtual void configure(NodePtr parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf, PathData* path) override final {
    nh_ = parent;
    name_ = name;
    tf_ = tf;
    pathData_ = path;
    auto peers = path->getPeers();
    asLeader_ = path->isLeader();
    for (const auto& p : peers) {
      // The names should be consistent with the ones in path_server_node.cpp
      // RCLCPP_DEBUG(nh_->get_logger(), p->getName().c_str());
      if (p->getName() == "left_boundary") {
        leftPeer_ = p;
        leftActiveSeg_ = leftPeer_->GET_PROPERTY(planning::ActiveSegment);
      } else if (p->getName() == "right_boundary") {
        rightPeer_ = p;
        rightActiveSeg_ = rightPeer_->GET_PROPERTY(planning::ActiveSegment);
      }
    }
    pathOffsetRateLimit_ = static_cast<float>(
        safeDeclareParameter<double>(nh_, name + ".lateral_offset_rate_limit", 0.1));
    frontOffsetPathTopic_ = safeDeclareParameter<std::string>(
        nh_, name + ".topic.look_ahead_offset_path", "path/offset_path/look_ahead");
    rearOffsetPathTopic_ = safeDeclareParameter<std::string>(
        nh_, name + ".topic.look_behind_offset_path", "path/offset_path/look_behind");
    frontOffsetArrayTopic_ = safeDeclareParameter<std::string>(
        nh_, name + ".topic.look_ahead_offset_values", "offset/look_ahead_values");
    rearOffsetArrayTopic_ = safeDeclareParameter<std::string>(
        nh_, name + ".topic.look_behind_offset_values", "offset/look_behind_values");
    offsetMaxTopic_ = safeDeclareParameter<std::string>(nh_, name + ".topic.offset_left_boundary",
                                                        "offset/left_bound");
    offsetMinTopic_ = safeDeclareParameter<std::string>(nh_, name + ".topic.offset_right_boundary",
                                                        "offset/right_bound");
    activeSegProperty_ = pathData_->GET_PROPERTY(planning::ActiveSegment);
    curveLength_ = pathData_->GET_PROPERTY(planning::CurveLength);
    curvature_ = pathData_->GET_PROPERTY(planning::Curvature);
  };

  virtual bool update() override final {
    if (curveLength_ == nullptr || activeSegProperty_ == nullptr || !curveLength_->isActive() ||
        !activeSegProperty_->isActive()) {
      frontOffsetPath_ = {pathData_->getClosestWaypoint()};
      rearOffsetPath_ = frontOffsetPath_;
      return false;
    }

    frontOffsetPath_ = activeSegProperty_->getFrontPath();
    rearOffsetPath_ = activeSegProperty_->getRearPath();

    // service rear offset array with index delta
    // moving forward, feed rear offset with the closest front offset
    const int& delta = pathData_->getCurrentIndexDelta();
    for (int n = 0; n < delta; n++) {
      rearOffset_.emplace_front(frontOffset_.size() ? frontOffset_.front() : 0.);
      if (frontOffset_.size() > 1) frontOffset_.pop_front();
    }
    // moving backward, drop the closest rear offset
    for (int n = 0; (n < -delta) && (rearOffset_.size() >= 1); n++) {
      rearOffset_.pop_front();
    }
    // resize rear offset to make alignment
    rearOffset_.resize(rearOffsetPath_.size(), rearOffset_.size() ? rearOffset_.back() : 0.);

    // service front offset array and pose array
    static auto pose_offset_fcn = [&](const float& offset, const PoseStamped& pose) {
      PoseStamped ret = pose;
      // offset pose to its left by offsetArray[i]. Use quaternion of the pose.
      tf2::Quaternion q;
      tf2::fromMsg(pose.pose.orientation, q);
      const tf2::Vector3 offsetVect = tf2::quatRotate(q, tf2::Vector3(0., offset, 0.));
      ret.pose.position.x += offsetVect.x();
      ret.pose.position.y += offsetVect.y();
      ret.pose.position.z += offsetVect.z();
      return ret;
    };

    const double& odometer = curveLength_->getOdometer();

    if (leftPeer_ != nullptr && rightPeer_ != nullptr) {
      if (!asLeader_ || leftActiveSeg_ == nullptr || rightActiveSeg_ == nullptr ||
          !leftActiveSeg_->isActive() || !rightActiveSeg_->isActive()) {
        // formulate feasible offset range. This implementation assumes a strict center path and
        // constant lane width.
        // In the published message, first element is max (left end), second element is min (right
        // end).
        // std::cout << "IN SINGLE-POINT UPDATE OF OFFSET BOUNDARY" << std::endl;
        const PoseStamped& leftBound = leftPeer_->getClosestWaypoint();
        Pose L = path_utils::safeProjection(*tf_, pathData_->getClosestWaypoint(), leftBound);
        offsetLeftBound_[0] = std::fmax(static_cast<float>(L.position.y), 0.);
        const PoseStamped& rightBound = rightPeer_->getClosestWaypoint();
        Pose R = path_utils::safeProjection(*tf_, pathData_->getClosestWaypoint(), rightBound);
        offsetRightBound_[0] = std::fmin(static_cast<float>(R.position.y), 0.);
      } else {
        // std::cout << "IN SYNCHRONIZED UPDATE OF OFFSET BOUNDARY" << std::endl;
        const std::vector<PoseStamped>& frontPath = activeSegProperty_->getFrontPath();
        size_t activeSegSize = frontPath.size();
        offsetLeftBound_.resize(activeSegSize);
        offsetRightBound_.resize(activeSegSize);
        const std::vector<PoseStamped>& leftBound = leftActiveSeg_->getFrontPath();
        const std::vector<PoseStamped>& rightBound = rightActiveSeg_->getFrontPath();
        for (size_t i = 0; i < activeSegSize; i++) {
          Pose L = path_utils::safeProjection(*tf_, frontPath[i], leftBound[i]);
          offsetLeftBound_[i] = std::fmax(static_cast<float>(L.position.y), 0.);
          Pose R = path_utils::safeProjection(*tf_, frontPath[i], rightBound[i]);
          offsetRightBound_[i] = std::fmin(static_cast<float>(R.position.y), 0.);
        }
      }
    }

    if (doUpcomingOffset_) {
      // if offset, offset the path per request.
      // offset message format:
      // header
      // float32 lateral_offset (m, global frenet, +x is left.)
      // float32 effective_from_distance (m, global frenet)
      // float32 effective_until_distance (m, global frenet)
      // ease in the path offset from the received point.
      int currInd = pathData_->getCurrentIndex();
      frontOffset_.clear();
      // Follow a ease-in -- hold -- ease-out pattern based on
      // odomMark_, odomSet_, odomLift, and odomClear_
      auto odomArray =
          curveLength_->getFrenetXDistanceArray(currInd, currInd + frontOffsetPath_.size() - 1);
      std::transform(odomArray.cbegin(), odomArray.cend(), odomArray.begin(),
                     [&odometer](const double& in) { return in + odometer; });
      for (size_t n = 0; n < frontOffsetPath_.size(); n++) {
        float nodeOffset = 0.0;
        if (odomArray[n] < pathOffsetOdomMark_)
          nodeOffset = existingOffset_;
        else if (odomArray[n] < pathOffsetOdomSet_) {
          float ratio = (odomArray[n] - pathOffsetOdomMark_) / pathOffsetEaseInDist_;
          nodeOffset = ratio * pathCmdOffsetLeft_ + (1. - ratio) * existingOffset_;
        } else if (odomArray[n] < pathOffsetOdomLift_)
          nodeOffset = pathCmdOffsetLeft_;
        else if (odomArray[n] < pathOffsetOdomClear_)
          nodeOffset = pathCmdOffsetLeft_ * (pathOffsetOdomClear_ - odomArray[n]) /
                       (pathOffsetOdomClear_ - pathOffsetOdomLift_);
        // else: nodeOffset = 0.0
        frontOffset_.emplace_back(nodeOffset);
      }
      // std::cout << odometer_ << " " << pathOffsetOdomMark_ << " " << pathOffsetOdomSet_ << " "
      //           << pathOffsetOdomLift_ << " " << pathOffsetOdomClear_ << "\n";

      /*
      // TODO: I doubt this is necessary. The heading change may be minimal.
      // In certain cases (e.g. a holonomic robot), we may not want the target heading to change at
      // all.
      double hdgCorrection = std::atan2(pathCmdOffsetLeft_, pathOffsetEaseInDist_);
      int i = 0;
      for (auto& pose : frontPath_.offsetPoses) {
        // add heading correction to the quaternion.
        if (frontPath_.offset_left[i] != pathCmdOffsetLeft_) {
          tf2::Quaternion q_corrected;
          q_corrected.setRPY(0., 0., hdgCorrection + q.getAngle());
          pose.pose.orientation = tf2::toMsg((q_corrected * q).normalize());
        }
        i++;
      }
      */

      std::transform(frontOffset_.cbegin(), frontOffset_.cend(), frontOffsetPath_.cbegin(),
                     frontOffsetPath_.begin(), pose_offset_fcn);

      // Reset path offset flag when the offset segment has passed
      if (odometer > pathOffsetOdomClear_) {
        doUpcomingOffset_ = false;
      }
    } else {
      frontOffset_.resize(frontOffsetPath_.size(), 0.);
    }

    if (doUpcomingOffset_ ||
        odometer < pathOffsetOdomClear_ + activeSegProperty_->getLookBehindDistance()) {
      std::transform(rearOffset_.cbegin(), rearOffset_.cend(), rearOffsetPath_.cbegin(),
                     rearOffsetPath_.begin(), pose_offset_fcn);
    }

    publish();
    return true;
  }

  void clearOffset() { doUpcomingOffset_ = false; }

  virtual void deactivate() override final {
    active_ = false;
    pathOffsetSub_.reset();
    clearOffset();
    frontOffsetPathPub_.reset();
    rearOffsetPathPub_.reset();
    frontOffsetValPub_.reset();
    rearOffsetValPub_.reset();
    offsetMaxPub_.reset();
    offsetMinPub_.reset();
  }

  virtual void activate() override final {
    frontOffset_ = {0.};
    rearOffset_ = {0.};
    frontOffsetPathPub_ = nh_->create_publisher<Path>(frontOffsetPathTopic_, 2);
    rearOffsetPathPub_ = nh_->create_publisher<Path>(rearOffsetPathTopic_, 2);
    frontOffsetValPub_ = nh_->create_publisher<Float32MultiArray>(frontOffsetArrayTopic_, 2);
    rearOffsetValPub_ = nh_->create_publisher<Float32MultiArray>(rearOffsetArrayTopic_, 2);
    offsetMaxPub_ = nh_->create_publisher<Float32MultiArray>(offsetMaxTopic_, 2);
    offsetMinPub_ = nh_->create_publisher<Float32MultiArray>(offsetMinTopic_, 2);
    if (curveLength_ != nullptr) {
      pathOffsetSub_ = nh_->create_subscription<PathOffsetCommand>(
          "path_offset_command", rclcpp::SystemDefaultsQoS(),
          [this](PathOffsetCommand::UniquePtr msg) {
            this->offsetPath(msg->effective_from_s, msg->effective_until_s,
                             msg->offset_y);
          });
    }
    active_ = true;
  }

  const std::deque<float>& getFrontOffsetValue() const { return frontOffset_; }

  const std::vector<PoseStamped>& getFrontOffsetPath() const { return frontOffsetPath_; }

  const std::deque<float>& getRearOffsetValue() const { return rearOffset_; }

  const std::vector<PoseStamped>& getRearOffsetPath() const { return rearOffsetPath_; }

  const std::vector<float>& getOffsetLeftBoundary() const { return offsetLeftBound_; }

  const std::vector<float>& getOffsetRightBoundary() const { return offsetRightBound_; }

  const bool& isOffsetActive() const { return doUpcomingOffset_; }

  template <typename T = double>
  std::vector<T> getActiveSegFrenetXDistanceArrayCompensated() const {
    std::vector<T> ret;
    if (doUpcomingOffset_ || curveLength_ == nullptr) {
      ret.resize(frontOffsetPath_.size(), 0.);
      double sum = 0.;
      std::transform(frontOffsetPath_.cbegin() + 1, frontOffsetPath_.cend(),
                     frontOffsetPath_.cbegin(), ret.begin() + 1,
                     [&sum](const PoseStamped& pose1, const PoseStamped& pose2) {
                       sum += std::hypot(pose1.pose.position.x - pose2.pose.position.x,
                                         pose1.pose.position.y - pose2.pose.position.y);
                       return static_cast<T>(sum);
                     });
    } else {
      int currInd = pathData_->getCurrentIndex();
      ret =
          curveLength_->getFrenetXDistanceArray<T>(currInd, currInd + frontOffsetPath_.size() - 1);
    }
    return ret;
  }

  template <typename T = float>
  std::vector<T> getActiveSegCurvatureArrayCompensated() const {
    std::vector<T> ret{};
    if (curvature_ == nullptr) return ret;
    if (doUpcomingOffset_) {
      auto curv = curvature_->recalculateUpcomingCurvature(frontOffsetPath_);
      ret = std::vector<T>{curv.cbegin(), curv.cend()};
    } else {
      ret = curvature_->getUpcomingCurvature<T>();
    }
    return ret;
  }

 protected:
  // called when a new offset command comes in.
  void offsetPath(const float& from, const float& to, const float& leftingAmount) {
    // TODO: consider timing delays from header
    existingOffset_ = frontOffset_[0];
    doUpcomingOffset_ = existingOffset_ != 0. || leftingAmount != 0.;
    pathOffsetOdomMark_ = curveLength_->getOdometer();
    // Limit lateral rate: the lane change is completed in no less than
    // delta/pathOffsetEaseInRate_ distance ahead, using linear lateral interp.
    float delta = leftingAmount - existingOffset_;
    // NOTE: 0.001 m (1 mm) is the relaxation term for calculating ease-in rate.
    pathOffsetEaseInDist_ = std::fmax(from, std::fabs(delta / pathOffsetRateLimit_)) + 0.001;
    // These two lines will ensure te specified leftingAmount be achieved, but risks running beyond
    // the end of the desired section.
    pathOffsetOdomSet_ = pathOffsetOdomMark_ + static_cast<double>(pathOffsetEaseInDist_);
    pathOffsetOdomLift_ =
        pathOffsetOdomMark_ + static_cast<double>(std::fmax(pathOffsetEaseInDist_, to)) + 0.001;
    // add a small number to prevent divide by zero
    pathOffsetOdomClear_ =
        pathOffsetOdomLift_ + std::fabs(leftingAmount / pathOffsetRateLimit_) + 0.001;
    pathCmdOffsetLeft_ = leftingAmount;
  }

  void publish() {
    auto front = std::make_unique<Path>();
    front->header.frame_id = pathData_->getTargetFrame();
    front->header.stamp = nh_->now();
    front->poses = frontOffsetPath_;
    frontOffsetPathPub_->publish(std::move(front));

    auto rear = std::make_unique<Path>();
    rear->header.frame_id = pathData_->getTargetFrame();
    rear->header.stamp = nh_->now();
    rear->poses = rearOffsetPath_;
    rearOffsetPathPub_->publish(std::move(rear));

    auto frontVal = std::make_unique<Float32MultiArray>();
    frontVal->data = {frontOffset_.cbegin(), frontOffset_.cend()};
    frontOffsetValPub_->publish(std::move(frontVal));

    auto rearVal = std::make_unique<Float32MultiArray>();
    rearVal->data = {rearOffset_.cbegin(), rearOffset_.cend()};
    rearOffsetValPub_->publish(std::move(rearVal));

    auto lbound = std::make_unique<Float32MultiArray>();
    lbound->data = offsetLeftBound_;
    offsetMaxPub_->publish(std::move(lbound));

    auto rbound = std::make_unique<Float32MultiArray>();
    rbound->data = offsetRightBound_;
    offsetMinPub_->publish(std::move(rbound));
  }

  NodePtr nh_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_ = nullptr;
  PathData* pathData_ = nullptr;
  bool asLeader_ = true;
  const PathData* leftPeer_ = nullptr;
  const PathData* rightPeer_ = nullptr;
  const ActiveSegment* leftActiveSeg_ = nullptr;
  const ActiveSegment* rightActiveSeg_ = nullptr;

  // just a placeholder... will be re-written upon receiving a new command.
  float pathOffsetEaseInDist_ = 50.0;
  float pathCmdOffsetLeft_ = 0.0;
  bool doUpcomingOffset_ = false;
  // offset control points
  double pathOffsetOdomMark_, pathOffsetOdomSet_, pathOffsetOdomLift_, pathOffsetOdomClear_;
  // offset value when the command is received, and the commanded value. Used to calculate ease-in.
  float existingOffset_ = 0.0;

  // limit on dist_lateral_travelled/dist_forward_travelled. This controls how abrupt the change is.
  float pathOffsetRateLimit_ = 0.1;

  std::vector<PoseStamped> frontOffsetPath_{}, rearOffsetPath_{};
  std::deque<float> frontOffset_{0.}, rearOffset_{0.};
  std::vector<float> offsetLeftBound_{0.}, offsetRightBound_{0.};

  std::string frontOffsetPathTopic_, rearOffsetPathTopic_, frontOffsetArrayTopic_,
      rearOffsetArrayTopic_, offsetMaxTopic_, offsetMinTopic_;

  ActiveSegment* activeSegProperty_;
  CurveLength* curveLength_;
  Curvature* curvature_;

  rclcpp::Publisher<Path>::SharedPtr frontOffsetPathPub_, rearOffsetPathPub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr frontOffsetValPub_, rearOffsetValPub_;
  // Front offset boundaries -- the "boundary array" implementation
  rclcpp::Publisher<Float32MultiArray>::SharedPtr offsetMaxPub_, offsetMinPub_;

  rclcpp::Subscription<PathOffsetCommand>::SharedPtr pathOffsetSub_ = nullptr;
};

}  // namespace planning

#endif
