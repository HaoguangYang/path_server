/**
 * @file curve_length.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief A path server plugin that manages the curve length along the frenet frame of the stored
 * path. It provides a coarse accountant for the distance travelled, as well as utilities extracting
 * the curve length between two sample points on the path.
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

#ifndef PATH_PROPERTY_CURVE_LENGTH_HPP
#define PATH_PROPERTY_CURVE_LENGTH_HPP

#include "path_server_overhaul/path_data.hpp"
#include "path_server_overhaul/path_property.hpp"
#include "path_server_overhaul/path_utils.hpp"

namespace planning {

using geometry_msgs::msg::PoseStamped;
using namespace path_utils;

class CurveLength : public PathProperty {
 public:
  virtual void configure(rclcpp::Node* parent, std::string name,
                         std::shared_ptr<tf2_ros::Buffer> tf, PathData* path) override final {
    nh_ = parent;
    name_ = name;
    tf_ = tf;
    pathData_ = path;
    initFrenetXDistance();
    RCLCPP_INFO(nh_->get_logger(), "Initialized property %s with length %f meters.", name_.c_str(),
                getPathCumulativeDist());
  };

  virtual bool update() override final {
    odometer_ += getFrenetXDistance(pathData_->getPreviousIndex(), pathData_->getCurrentIndex());
    return true;
  }

  /**
   * @brief This function returns the frenet-frame distance between two indices on the
   * stored path (using milestone_). The distance is directional, from ind1 to ind2.
   * If the path is closed, this function allows the two numbers to exceed the
   * size limit of the stored path. The corresponding point is found using modulo.
   *
   * @param ind1 starting index
   * @param ind2 ending index
   * @return double distance along the frenet frame of the stored path
   */
  double getFrenetXDistance(const int& ind1, const int& ind2) const {
    double ret;
    if (!milestone_.size()) {
      ret = 0.0;
      return ret;
    }
    if (ind1 == ind2) {
      ret = 0.0;
      return ret;
    }
    if (!pathData_->isClosedPath()) {
      // direct subtraction from the cumulative distance vector.
      ret = milestone_[std::clamp(ind2, 0, static_cast<int>(milestone_.size()) - 1)] -
            milestone_[std::clamp(ind1, 0, static_cast<int>(milestone_.size()) - 1)];
      return ret;
    }
    /* we are running a closed path. Need to consider crossing the starting point.
     * Hence, we allow the indices to exceed poses.size(), indicating running along
     * the specified direction.
     * */
    int laps =
        ind2 / static_cast<int>(milestone_.size()) - ind1 / static_cast<int>(milestone_.size());
    int rem1 = ind1 % static_cast<int>(milestone_.size());
    int rem2 = ind2 % static_cast<int>(milestone_.size());
    if (rem1 < 0) {
      rem1 += milestone_.size();
      laps++;
    }
    if (rem2 < 0) {
      rem2 += milestone_.size();
      laps--;
    }
    ret = milestone_[0] * laps + (rem2 ? milestone_[rem2] : 0.0) - (rem1 ? milestone_[rem1] : 0.0);
    return ret;
  }

  /**
   * @brief This function returns the frenet-frame distance vector, of EVERY POINT between two
   * indices on the stored path (using milestone_). The distance is directional, from ind1 to ind2.
   * If the path is closed, this function allows the two numbers to exceed the
   * size limit of the stored path. The corresponding point is found using modulo.
   *
   * @tparam T return data type in the vector
   * @param ind1 starting index
   * @param ind2 ending index
   * @return std::vector<T> distance along the frenet frame of the stored path at each point [ind1,
   * ind2] in sequence.
   */
  template <typename T = double>
  std::vector<T> getFrenetXDistanceArray(const int& ind1, const int& ind2) const {
    std::vector<T> ret(std::abs(ind2 - ind1 + 1));
    ret[0] = 0.;
    if (ind2 == ind1) return ret;
    if (ind2 > ind1) {
      for (int i = ind1 + 1; i <= ind2; i++) {
        ret[i - ind1] = getFrenetXDistance(ind1, i);
      }
    } else {
      for (int i = ind1 - 1; i >= ind2; i--) {
        ret[ind1 - i] = getFrenetXDistance(ind1, i);
      }
    }
    return ret;
  }

  /**
   * @brief Get the path's cumulative distance (in meters for lat/lon, or the path file unit)
   *
   * @return double
   */
  double getPathCumulativeDist() const {
    return pathData_->isClosedPath() ? milestone_.front() : milestone_.back();
  }

  /**
   * @brief Get the odometer reading from counting the passed indices on the stored path.
   *
   * @return const double& reference to the odometer reading.
   */
  const double& getOdometer() const { return odometer_; }

  /**
   * @brief Get the laps traveled since starting.
   *
   * @return double Laps as a floating point, from the odometer reading divided by the lap length.
   */
  double getLaps() const { return getOdometer() / getPathCumulativeDist(); }

  const std::vector<double>& getMilestone() { return milestone_; }

 protected:
  /**
   * @brief initializer for milestone_ vector. Used for frenet frame distance
   * calculation.
   */
  void initFrenetXDistance() {
    PathCandidate* globPath = pathData_->getGlobalPath();
    // TODO: if frenet cumulative distance is already the latest version, skip re-initialization.
    milestone_.clear();
    milestone_.reserve(globPath->size());
    milestone_.emplace_back(0.0);
    for (auto it = globPath->cbegin(); it < globPath->cend() - 1; it++) {
      // std::cout << milestone_.back() << std::endl;
      //  get distance between this and next poses.
      milestone_.emplace_back(milestone_.back() + safeGetDistance(*tf_, *it, *(it + 1)));
    }
    if (pathData_->isClosedPath()) {
      milestone_[0] =
          (milestone_.back() + safeGetDistance(*tf_, globPath->back(), globPath->front()));
    }
    // verify
    if (milestone_[0] < 0.) milestone_.clear();
  }

  rclcpp::Node* nh_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_ = nullptr;
  PathData* pathData_ = nullptr;
  std::vector<double> milestone_{};
  double odometer_ = 0.;
};

}  // namespace planning

#endif
