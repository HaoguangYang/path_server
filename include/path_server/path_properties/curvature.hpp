/**
 * @file curvature.hpp
 * @author Haoguang YANG (yang1510@purdue.edu)
 * @brief A path server plugin that calculates and manages curvature property of a path segment
 * @version 0.1
 * @date 2023-07-08
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

#ifndef PATH_PROPERTY_CURVATURE_HPP
#define PATH_PROPERTY_CURVATURE_HPP

#include <deque>

#include "path_server/path_data.hpp"
#include "path_server/path_properties/active_segment.hpp"
#include "path_server/path_property.hpp"
#include "path_server/path_utils.hpp"

namespace planning {

using geometry_msgs::msg::PoseStamped;

class Curvature : public PathProperty {
 public:
  virtual void configure(NodePtr parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf, PathData* path) override final {
    nh_ = parent;
    name_ = name;
    tf_ = tf;
    pathData_ = path;
    activeSegProperty_ = pathData_->GET_PROPERTY(planning::ActiveSegment);
    initCurvature();
  };

  /**
   * @brief initializer for curvature_ vector. Used to bake curvature of interpolated paths.
   */
  void initCurvature() {
    globCurvature_.clear();
    PathCandidate* globalPath = pathData_->getGlobalPath();
    globCurvature_.reserve(globalPath->size());
    // head
    if (pathData_->isClosedPath()) {
      globCurvature_.emplace_back(path_utils::mengerCurvature(
          globalPath->back(), globalPath->front(), *(globalPath->cbegin() + 1)));
    } else {
      globCurvature_.emplace_back(0.0);
    }
    // body
    for (auto it = globalPath->cbegin(); it < globalPath->cend() - 2; it++) {
      // get curvature at this point.
      globCurvature_.emplace_back(path_utils::mengerCurvature(*it, *(it + 1), *(it + 2)));
    }
    // std::cout << curvature_.back() << std::endl;
    // tail
    if (pathData_->isClosedPath()) {
      globCurvature_.emplace_back(path_utils::mengerCurvature(
          *(globalPath->cend() - 2), globalPath->back(), globalPath->front()));
    } else {
      globCurvature_.emplace_back(globCurvature_.back());
    }
  }

  virtual bool update() override final {
    // Transform-invariant fields
    // update curvature
    const int currInd = pathData_->getCurrentIndex();
    if (activeSegProperty_ == nullptr || !activeSegProperty_->isActive()) {
      frontSegCurv_ = {globCurvature_[currInd]};
      rearSegCurv_ = {globCurvature_[currInd]};
    } else {
      // service rear curvature array with index delta
      const int deltaInd = pathData_->getCurrentIndexDelta();
      // moving forward, feed rear curvature with the closest front curvature
      for (int n = 0; n < deltaInd; n++) {
        rearSegCurv_.emplace_front(frontSegCurv_.size() ? frontSegCurv_.front() : 0.);
        if (frontSegCurv_.size() > 1) frontSegCurv_.pop_front();
      }
      // moving backward, drop the closest rear curvature. This usually should not happen.
      for (int n = 0; (n < -deltaInd) && (rearSegCurv_.size() >= 1); n++) {
        rearSegCurv_.pop_front();
      }
      // resize rear curvature to make alignment
      const int lookBehindInd = activeSegProperty_->getLookBehindIndex();
      rearSegCurv_.resize(currInd - lookBehindInd + 1,
                          rearSegCurv_.size() ? rearSegCurv_.back() : 0.);
      // service front curvature array
      const int lookAheadInd = activeSegProperty_->getLookAheadIndex();
      auto curv_tmp = path_utils::getActiveSeg(globCurvature_, currInd, lookAheadInd,
                                               pathData_->isClosedPath());
      frontSegCurv_ = {curv_tmp.cbegin(), curv_tmp.cend()};
    }
    return true;
  }

  std::deque<float>& recalculateUpcomingCurvature(const std::vector<PoseStamped>& newPath) {
    /*
    if (pathOffsetProperty_ != nullptr && pathOffsetProperty_->isActive() &&
        pathOffsetProperty_->isOffsetActive()) {
      // treat curvature array
      // FIXME: this is a HACK -- in fact, curvatures at all points need recalculation.
      auto frontOffsetLeft = pathOffsetProperty_->getFrontOffsetValue();
      std::transform(frontOffsetLeft.cbegin(), frontOffsetLeft.cend(), frontSegCurv_.cbegin(),
                     frontSegCurv_.begin(), [](const float& offset, const float& curvature) {
                       return (fabs(curvature) > 1.e-5 && offset != 0.)
                                  ? 1. / (1. / curvature - offset)
                                  : curvature;
                     });
      auto frontOffsetPath = pathOffsetProperty_->getFrontOffsetPath();
    */
    if (newPath.size() < 3) return frontSegCurv_;
    size_t updateRange = std::min(frontSegCurv_.size(), newPath.size());
    for (size_t ind = 1; ind < updateRange - 1; ind++) {
      // get curvature at this point.
      frontSegCurv_[ind] = static_cast<float>(
          path_utils::mengerCurvature(newPath[ind - 1], newPath[ind], newPath[ind + 1]));
    }
    // Apply median filter of window 3 to calculated local curvature to get rid of spikes.
    auto curvTmp = frontSegCurv_;
    for (size_t ind = 1; ind < updateRange - 1; ind++) {
      float r, s;
      if (frontSegCurv_[ind - 1] > frontSegCurv_[ind + 1]) {
        r = frontSegCurv_[ind - 1];
        s = frontSegCurv_[ind + 1];
      } else {
        r = frontSegCurv_[ind + 1];
        s = frontSegCurv_[ind - 1];
      }
      // r is the larger number
      if (frontSegCurv_[ind] < s)
        curvTmp[ind] = s;
      else if (frontSegCurv_[ind] > r)
        curvTmp[ind] = r;
    }
    frontSegCurv_.swap(curvTmp);
    return frontSegCurv_;
  }

  const std::vector<float>& getGlobalCurvature() const { return globCurvature_; }

  template <typename T = float>
  std::vector<T> getUpcomingCurvature() const {
    return std::vector<T>{frontSegCurv_.cbegin(), frontSegCurv_.cend()};
  }

  template <typename T = float>
  std::vector<T> getPastCurvature() const {
    return std::vector<T>{rearSegCurv_.cbegin(), rearSegCurv_.cend()};
  }

 protected:
  NodePtr nh_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_ = nullptr;
  PathData* pathData_ = nullptr;

  std::vector<float> globCurvature_{};
  std::deque<float> frontSegCurv_, rearSegCurv_;

  ActiveSegment* activeSegProperty_;
};

}  // namespace planning

#endif
