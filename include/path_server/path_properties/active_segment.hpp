/**
 * @file active_segment.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief A path server plugin that extracts a segment from the global path. The segment is centered
 * at the closest point from the robot to the global path, and extends forwards and backwards up to
 * a look-ahead and look-behind distance, respectively.
 * @version 0.1
 * @date 2023-07-07
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

#ifndef PATH_PROPERTY_ACTIVE_SEGMENT_HPP
#define PATH_PROPERTY_ACTIVE_SEGMENT_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_server/path_data.hpp"
#include "path_server/path_properties/curve_length.hpp"
#include "path_server/path_property.hpp"
#include "path_server/path_utils.hpp"

namespace planning {

using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using namespace path_utils;

class ActiveSegment : public PathProperty {
 public:
  virtual void configure(NodePtr parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf, PathData* path) override final {
    nh_ = parent;
    name_ = name;
    tf_ = tf;
    pathData_ = path;
    asLeader_ = path->isLeader();
    if (!asLeader_) leaderActiveSeg_ = path->getLeader()->GET_PROPERTY(planning::ActiveSegment);
    lookAheadDist_ = safeDeclareParameter<double>(nh_, name + ".look_ahead_distance", 100.0);
    lookBehindDist_ = safeDeclareParameter<double>(nh_, name + ".look_behind_distance", 0.0);
    frontPathTopic_ =
        safeDeclareParameter<std::string>(nh_, name + ".topic.look_ahead_path", "path/look_ahead");
    rearPathTopic_ = safeDeclareParameter<std::string>(nh_, name + ".topic.look_behind_path",
                                                       "path/look_behind");
    curveLength_ = pathData_->GET_PROPERTY(planning::CurveLength);
  };

  virtual bool update() override final {
    // Now, generate the pose list from currentInd_, extending forward and backward.
    updateLookAheadAndBehindInds();

    // Transform-sensitive fields
    // transform poses. This needs to be done every cycle.

    // update active path segments from stored path representations.
    if (lookAheadDist_ <= 0.)
      frontPath_ = {pathData_->getClosestWaypoint()};
    else {
      frontPath_ = safeTransformPoses(
          *tf_, pathData_->getTargetFrame(),
          getActiveSeg(pathData_->getGlobalPath()->poses, pathData_->getCurrentIndex(),
                       lookAheadInd_, pathData_->isClosedPath()));
    }

    if (lookBehindDist_ <= 0.)
      rearPath_ = {pathData_->getClosestWaypoint()};
    else {
      rearPath_ = safeTransformPoses(
          *tf_, pathData_->getTargetFrame(),
          getActiveSeg(pathData_->getGlobalPath()->poses, pathData_->getCurrentIndex(),
                       lookBehindInd_, pathData_->isClosedPath()));
    }

    if (!frontPath_.size() || !rearPath_.size()) return false;

    publishPaths();
    return true;
  }

  virtual void deactivate() override final {
    active_ = false;
    frontPathPub_.reset();
    rearPathPub_.reset();
  }

  virtual void activate() override final {
    frontPathPub_ = nh_->create_publisher<Path>(frontPathTopic_, 5);
    rearPathPub_ = nh_->create_publisher<Path>(rearPathTopic_, 5);
    active_ = true;
  }

  const int& getLookAheadIndex() const { return lookAheadInd_; }
  const int& getLookBehindIndex() const { return lookBehindInd_; }

  const double& getLookAheadDistance() const { return lookAheadDist_; }
  const double& getLookBehindDistance() const { return lookBehindDist_; }

  const std::vector<PoseStamped>& getFrontPath() const { return frontPath_; }
  const std::vector<PoseStamped>& getRearPath() const { return rearPath_; }

 protected:
  /**
   * @brief update indices where the path is cut off at front and rear directions.
   * The resultant indices are different from the current (nearest) path pose index,
   * unless it is one of the starting or ending poses.
   * The resultant indices follow path directions when seen at currentInd_, i.e.
   * if the path is closed, the lookForwardInd_ may exceed path size, and
   * the lookBehindInd_ may be negative. These indices are unwrapped values.
   */
  void updateLookAheadAndBehindInds() {
    if (!asLeader_) {
      lookAheadInd_ = leaderActiveSeg_->getLookAheadIndex();
      lookBehindInd_ = leaderActiveSeg_->getLookBehindIndex();
      return;
    }
    double trialDist;
    int currentInd = pathData_->getCurrentIndex();
    int pathSize = static_cast<int>(pathData_->getGlobalPath()->size());
    // update lookAheadInd_ and lookBehindInd_ by searching for the first index exceeding or on the
    // distance bounds
    if (lookAheadDist_ <= 0.)
      lookAheadInd_ = currentInd;
    else {
      if (lookAheadInd_ <= currentInd) {
        lookAheadInd_ = std::min(currentInd + 1, pathSize - 1);
      }
      trialDist = curveLength_->getFrenetXDistance(currentInd, lookAheadInd_);
      // find lookAheadInd_: first index that exceeds lookAheadDistance from currentInd_.
      if (trialDist > lookAheadDist_) {
        /*        closer <--- LookAhead distance ---> farther away
         *                             <-|--- Approaching from this side
         */
        do {
          lookAheadInd_--;
          if (lookAheadInd_ == currentInd) break;
          trialDist = curveLength_->getFrenetXDistance(currentInd, lookAheadInd_);
        } while (trialDist > lookAheadDist_);
        // exactly one index over or on the distance bounds.
        if (trialDist < lookAheadDist_) lookAheadInd_ = std::min(lookAheadInd_ + 1, pathSize - 1);
      } else if (trialDist < lookAheadDist_) {
        /*        closer <--- LookAhead distance ---> farther away
         * Approaching from this side ---|->
         */
        do {
          // additional break condition when the path is not closed
          if (!pathData_->isClosedPath() && lookAheadInd_ >= pathSize - 1) {
            lookAheadInd_ = pathSize - 1;
            break;
          }
          lookAheadInd_++;
          trialDist = curveLength_->getFrenetXDistance(currentInd, lookAheadInd_);
        } while (trialDist < lookAheadDist_);
      }
    }

    if (lookBehindDist_ <= 0.) {
      lookBehindInd_ = currentInd;
    } else {
      if (lookBehindInd_ >= currentInd) {
        lookBehindInd_ = std::max(currentInd - 1, 0);
      }
      trialDist = curveLength_->getFrenetXDistance(lookBehindInd_, currentInd);
      // find lookBehindInd_: first index that exceeds lookBehindDistance from currentInd_.
      if (trialDist > lookBehindDist_) {
        /*        closer <---  LookBehind distance ---> farther away
         *                             <-|--- Approaching from this side
         */
        do {
          lookBehindInd_++;
          if (lookBehindInd_ == currentInd) break;
          trialDist = curveLength_->getFrenetXDistance(lookBehindInd_, currentInd);
        } while (trialDist > lookBehindDist_);
        // exactly one index over or on the distance bounds.
        if (trialDist < lookBehindDist_) lookBehindInd_ = std::max(lookBehindInd_ - 1, 0);
      } else if (trialDist < lookBehindDist_) {
        /*        closer <---  LookBehind distance ---> farther away
         * Approaching from this side ---|->
         */
        do {
          // additional break condition when the path is not closed
          if (!pathData_->isClosedPath() && lookBehindInd_ <= 0) {
            lookBehindInd_ = 0;
            break;
          }
          lookBehindInd_--;
          trialDist = curveLength_->getFrenetXDistance(lookBehindInd_, currentInd);
        } while (trialDist < lookBehindDist_);
      }
    }
  }

  void publishPaths() {
    auto front = std::make_unique<Path>();
    front->header.frame_id = pathData_->getTargetFrame();
    front->header.stamp = nh_->now();
    front->poses = frontPath_;
    frontPathPub_->publish(std::move(front));

    auto rear = std::make_unique<Path>();
    rear->header.frame_id = pathData_->getTargetFrame();
    rear->header.stamp = nh_->now();
    rear->poses = rearPath_;
    rearPathPub_->publish(std::move(rear));
  }

  NodePtr nh_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_ = nullptr;
  PathData* pathData_ = nullptr;
  bool asLeader_ = true;
  const ActiveSegment* leaderActiveSeg_ = nullptr;
  double lookAheadDist_ = 0., lookBehindDist_ = 0.;
  int lookAheadInd_ = 0, lookBehindInd_ = 0;
  std::vector<PoseStamped> frontPath_{}, rearPath_{};

  rclcpp::Publisher<Path>::SharedPtr frontPathPub_, rearPathPub_;
  std::string frontPathTopic_, rearPathTopic_;

  CurveLength* curveLength_;
};

}  // namespace planning

#endif
