/**
 * @file path_utils.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Utilities for manipulation and calculation related to paths and coordinates. Heavily based
 * on TF2 infrastructure.
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

#ifndef PATH_UTILS_HPP_
#define PATH_UTILS_HPP_

#include <cassert>
#include <cmath>
#include <vector>

#if __has_include("rclcpp/rclcpp.hpp")
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#elif __has_include("ros/ros.h")
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#endif

#include "tf2_ros/buffer.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#ifdef ROSCPP_ROS_H
namespace tf2 {
  typedef ros::Time TimePoint;
  const ros::Time TimePointZero = ros::Time(0);
}
#endif

namespace path_utils {

#ifdef RCLCPP__RCLCPP_HPP_
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
#elif defined ROSCPP_ROS_H
using geometry_msgs::PoseStamped;
using geometry_msgs::Pose;
#endif

/**
 * @brief This method extracts the segment [indStart, indEnd] (inclusive) from source. It accounts
 * for cyclic boundary.
 *
 * @tparam T array datatype
 * @param source source array
 * @param indStart starting index of the sub-array
 * @param indEnd ending index (inclusive) of the sub-array
 * @param isClosed consideration of cyclic boundary
 * @return std::vector<T> extracted sub-array.
 */
template <typename T>
inline std::vector<T> getActiveSeg(const std::vector<T>& source, const int& indStart,
                                   const int& indEnd, const bool& isClosed) {
  std::vector<T> target{};

  if (indStart == indEnd) {
    target = {source[indStart]};
    return target;
  }

  if (!isClosed) {
    if (indStart >= static_cast<int>(source.size()) || indStart < 0 ||
        indStart >= static_cast<int>(source.size()) || indStart < 0) {
      return target;
    }
  }

  int ind1 = indStart;
  int ind2 = indEnd;
  if (isClosed) {
    ind1 %= static_cast<int>(source.size());
    if (ind1 < 0) ind1 += source.size();
    ind2 %= static_cast<int>(source.size());
    if (ind2 < 0) ind2 += source.size();
  }

  if (indStart < indEnd) {
    target.reserve(indEnd - indStart + 1);
    // going positive direction
    if (ind1 > ind2) {
      target = {source.cbegin() + ind1, source.cend()};
      target.insert(target.end(), source.cbegin(), source.cbegin() + 1 + ind2);
    } else {
      target = {source.cbegin() + ind1, source.cbegin() + 1 + ind2};
    }
  } else {
    target.reserve(indStart - indEnd + 1);
    // going negative direction
    if (ind1 < ind2) {
      target = {source.crend() - 1 - ind1, source.crend()};
      target.insert(target.end(), source.crbegin(), source.crend() - ind2);
    } else {
      target = {source.crend() - 1 - ind1, source.crend() - ind2};
    }
  }
  return target;
}

/**
 * @brief Return the three-point Menger curvature based on their coordinates.
 *
 * @param p1 Point A
 * @param p2 Point B
 * @param p3 Point C
 * @return float curvature from A->B->C, expressed as 1/r. Curving left (+X->+Y) is positive.
 */
inline double mengerCurvature(const PoseStamped& p1, const PoseStamped& p2, const PoseStamped& p3) {
  if (p1.header.frame_id != p2.header.frame_id || p2.header.frame_id != p3.header.frame_id)
    throw std::runtime_error(
        "All points passed in for curvature calculation must be expressed in the same refrence "
        "frame.");
  double TrigA =
      (p2.pose.position.x - p1.pose.position.x) * (p3.pose.position.y - p1.pose.position.y) -
      (p2.pose.position.y - p1.pose.position.y) * (p3.pose.position.x - p1.pose.position.x);
  // left turn has positive curvature
  return 4. * TrigA /
         (1.e-12 + std::sqrt((std::pow(p2.pose.position.x - p1.pose.position.x, 2) +
                              std::pow(p2.pose.position.y - p1.pose.position.y, 2)) *
                             (std::pow(p3.pose.position.x - p2.pose.position.x, 2) +
                              std::pow(p3.pose.position.y - p2.pose.position.y, 2)) *
                             (std::pow(p3.pose.position.x - p1.pose.position.x, 2) +
                              std::pow(p3.pose.position.y - p1.pose.position.y, 2))));
};

/**
 * @brief a power tool to select the first member within a list that has a user-defined property.
 *
 * @tparam Lambda A Lambda function that returns a boolean value.
 * @tparam T The type of the candidates in the candidate set.
 * @tparam Args Argument types to be passed into the user-defined property.
 * @param candidates An initializer list representing the candidate set.
 * @param method The user-defined selection method. The first candidate with method(args) == true
 * will be returned.
 * @param args Arguments to be passed into the user-defined selection method.
 * @return T copy of the first candidate that meets the condition.
 * @throw std::runtime_error if no candidate meets the condition.
 */
template <typename Lambda, typename T, typename... Args>
inline T findFirstThat(const std::initializer_list<T>& candidates, const Lambda&& method,
                       const Args&... args) {
  for (const auto& item : candidates) {
    if (method(item, args...)) {
      return item;
    }
  }
  throw std::runtime_error("None of the provided candidates meet the conditions.");
}

/**
 * @brief A helper function to find the first candidate in the list that has size() > 0.
 *
 * @tparam T candidate type. Can be a vector-like STL type, or any instance that has a size()
 * method.
 * @param candidates initializer list of candidates
 * @return T copy of the candidate with size() call returning a positive value.
 * @throw std::runtime_error if no candidate has size() > 0.
 */
template <typename T>
inline T firstAvailable(const std::initializer_list<T>& candidates) {
  return findFirstThat(candidates, [](const T& item) { return (item->size() > 0); });
}

/**
 * @brief Test if the provided frame ids are transformable.
 *
 * @param tfBuffer transform buffer instance reference
 * @param to target frame id
 * @param from source frame id
 * @param time time point where the transform is desired. Defaults to latest (0).
 * @return true the transformation is feasible
 * @return false the transformation is infeasible
 */
inline bool safeCanTransform(const tf2_ros::Buffer& tfBuffer, const std::string& to,
                             const std::string& from,
                             const tf2::TimePoint& time = tf2::TimePointZero) noexcept {
  if (!(tfBuffer._frameExists(to) && tfBuffer._frameExists(from))) return false;
  return (tfBuffer.canTransform(to, from, time));
}

/**
 * @brief Obtain the 3D vector from pose1 to pose2, expressed in pose1.
 *
 * @param tfBuffer transform buffer instance reference
 * @param pose1 source pose and the resultant reference frame
 * @param pose2 target pose and the target's reference frame
 * @return std::tuple<double, double, double> 3D vector returned as a tuple of doubles. Returns NANs
 * if the transform is infeasible.
 */
inline std::tuple<double, double, double> safeVector(const tf2_ros::Buffer& tfBuffer,
                                                     const PoseStamped& pose1,
                                                     const PoseStamped& pose2) noexcept {
  double dx, dy, dz;
  if (pose1.header.frame_id != pose2.header.frame_id) {
    if (!safeCanTransform(tfBuffer, pose1.header.frame_id, pose2.header.frame_id,
                          tf2::TimePointZero)) {
      return std::make_tuple(NAN, NAN, NAN);
    }
    PoseStamped pose_alt = tfBuffer.transform(pose2, pose1.header.frame_id);
    dx = pose_alt.pose.position.x - pose1.pose.position.x;
    dy = pose_alt.pose.position.y - pose1.pose.position.y;
    dz = pose_alt.pose.position.z - pose1.pose.position.z;
  } else {
    dx = pose2.pose.position.x - pose1.pose.position.x;
    dy = pose2.pose.position.y - pose1.pose.position.y;
    dz = pose2.pose.position.z - pose1.pose.position.z;
  }
  return std::make_tuple(dx, dy, dz);
}

/**
 * @brief gets distance between two poses with frame_id handling.
 *
 * @param tfBuffer transform buffer instance reference
 * @param pose1 source pose and source reference frame
 * @param pose2 target pose and target's reference frame
 * @return double euclidean distnace between pose1 and pose2. Returns -1 if the two frames are not
 * transformable.
 */
inline double safeGetDistance(const tf2_ros::Buffer& tfBuffer, const PoseStamped& pose1,
                              const PoseStamped& pose2) noexcept {
  auto vect = safeVector(tfBuffer, pose1, pose2);
  if (isnan(std::get<0>(vect)) || isnan(std::get<1>(vect)) || isnan(std::get<2>(vect))) return -1.;
  return std::hypot(std::get<0>(vect), std::get<1>(vect), std::get<2>(vect));
}

/**
 * @brief transform path.poses using tf2 without throwing exceptions.
 *
 * @param tfBuffer transform buffer instance reference
 * @param outFrame frame id for the output result
 * @param inPoses input vector of poses
 * @return std::vector<PoseStamped> a vector of transformed poses. Any input poses that are not
 * transformable are automatically left out. Returns an empty vector if non of the input poses are
 * transformable.
 */
inline std::vector<PoseStamped> safeTransformPoses(
    const tf2_ros::Buffer& tfBuffer, const std::string& outFrame,
    const std::vector<PoseStamped>& inPoses) noexcept {
  std::vector<PoseStamped> outPoses;
  if (safeCanTransform(tfBuffer, outFrame, inPoses[0].header.frame_id, tf2::TimePointZero)) {
    const auto tf =
        tfBuffer.lookupTransform(outFrame, inPoses[0].header.frame_id, tf2::TimePointZero);
    outPoses.resize(inPoses.size());
    std::transform(inPoses.cbegin(), inPoses.cend(), outPoses.begin(),
                   [&](const PoseStamped& data) {
                     PoseStamped ret;
                     tf2::doTransform(data, ret, tf);
                     return ret;
                   });
  }
  return outPoses;
}

/**
 * @brief observe pose2 from pose1
 *
 * @param tfBuffer transform buffer instance reference
 * @param pose1 observer pose and the observer's reference frame.
 * @param pose2 observed item's pose in some reference frame.
 * @return Pose the observed relative pose of pose2, expressed in pose1.
 */
inline Pose safeProjection(const tf2_ros::Buffer& tfBuffer, const PoseStamped& pose1,
                           const PoseStamped& pose2) noexcept {
  Pose res;
  PoseStamped pose_alt = pose2;
  if (pose1.header.frame_id != pose2.header.frame_id) {
    if (!path_utils::safeCanTransform(tfBuffer, pose1.header.frame_id, pose2.header.frame_id,
                                      tf2::TimePointZero)) {
      res.position.x = NAN;
      res.position.y = NAN;
      res.position.z = NAN;
      res.orientation.w = NAN;
      res.orientation.x = 0.0;
      res.orientation.y = 0.0;
      res.orientation.z = 0.0;
      return res;
    }
    PoseStamped pose_alt = tfBuffer.transform(pose2, pose1.header.frame_id);
  }
  tf2::Quaternion q;
  tf2::fromMsg(pose1.pose.orientation, q);
  // express the difference w.r.t. pose1.
  q = q.inverse();
  const tf2::Vector3 v_offset =
      tf2::quatRotate(q, tf2::Vector3(pose_alt.pose.position.x - pose1.pose.position.x,
                                      pose_alt.pose.position.y - pose1.pose.position.y,
                                      pose_alt.pose.position.z - pose1.pose.position.z));
  res.position.x = v_offset.x();
  res.position.y = v_offset.y();
  res.position.z = v_offset.z();
  tf2::Quaternion q2;
  tf2::fromMsg(pose_alt.pose.orientation, q2);
  res.orientation = tf2::toMsg((q * q2).normalize());
  return res;
}

/**
 * @brief direction to pose 2 from pose1, measured in pose1 frame.
 *
 * @param pose1 observer's pose and observer's referece frame.
 * @param pose2 target's pose and target's reference frame.
 * @return tf2::Quaternion relative rotation from pose1 to pose2.
 */
inline tf2::Quaternion safeGetOrientation(const tf2_ros::Buffer& tfBuffer, const PoseStamped& pose1,
                                          const PoseStamped& pose2) noexcept {
  tf2::Quaternion q;
  auto vect = path_utils::safeVector(tfBuffer, pose1, pose2);
  if (isnan(std::get<0>(vect)) || isnan(std::get<1>(vect)) || isnan(std::get<2>(vect))) {
    // throw std::runtime_error("Vector could not be obtained between path poses.");
    return q.getIdentity();
  }
  double yaw = std::atan2(std::get<1>(vect), std::get<0>(vect));
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

/**
 * @brief Declares static ROS2 parameter and sets it to a given value if it was not already
 * declared.
 *
 * @tparam RetT return data type
 * @tparam NodePtrT node pointer data type
 * @param node pointer to a ROS2 node in which given parameter is to be declared
 * @param param_name name of the parameter to declare or find
 * @param default_value parameter value to initialize with
 * @param parameter_descriptor parameter descriptor (optional)
 * @return RetT return the parameter's value (either the initialized value, or the configured value)
 */
template <typename RetT, typename NodePtrT>
inline RetT safeDeclareParameter(
    NodePtrT node, const std::string& param_name, const RetT& default_value
#ifdef RCLCPP__RCLCPP_HPP_
    , const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor =
        rcl_interfaces::msg::ParameterDescriptor()
#endif
) noexcept {
  RetT ret;
#ifdef RCLCPP__RCLCPP_HPP_
  if (!node->has_parameter(param_name)) {
    ret = node->declare_parameter(param_name, default_value, parameter_descriptor);
  } else {
    rclcpp::Parameter param = node->get_parameter(param_name);
    ret = param.get_value<RetT>();
  }
#elif defined ROSCPP_ROS_H
  if (!node->hasParam(param_name)) {
    node->setParam(param_name, default_value);
  }
  node->getParam(param_name, ret);
#endif
  return ret;
}

/**
 * @brief This function finds the index on the path that is closest to the current
 * position. The returned index is in the range of [0, poses.size()]
 *
 * @param logger  The node->get_logger() return type to log outputs
 * @param tfBuffer  The tf2_ros::Buffer instance
 * @param poses The vector of poses to which the distance from self is calculated
 * @param self  The origin of distance calculation
 * @param startInd  Starting from this index in the provided pose vector
 * @param stopAtFirstMin  Whether to stop searching when a first minima is found
 * @param wrapAround  Whether to re-start search from the beginning once reaching the end
 * @return int  The index of the closest pose.
 */
#ifdef RCLCPP__RCLCPP_HPP_
typedef rclcpp::Logger Logger;
#elif defined ROSCPP_ROS_H
typedef void* Logger;
#endif
inline int findClosestInd(Logger logger, const tf2_ros::Buffer& tfBuffer,
                          const std::vector<PoseStamped>& poses, const PoseStamped& self,
                          const int& startInd, bool stopAtFirstMin, bool wrapAround) {
  int minId = startInd;
  double minDistance = DBL_MAX;
  bool found = false;
  // find next closest point from startInd towards the end
  for (int trialId = startInd; trialId < static_cast<int>(poses.size()); trialId++) {
    double thisDist = path_utils::safeGetDistance(tfBuffer, poses[trialId], self);
    if (thisDist < 0.) {
      // something went wrong.
      RCLCPP_ERROR(logger, "INITIALIZATION FAILED due to failed to get distance to path poses.");
      return -1;
    } else if (thisDist < minDistance) {
      minDistance = thisDist;
      minId = trialId;
    } else if (stopAtFirstMin) {
      // distance is increasing. Quit.
      found = true;
      break;
    }
  }
  if (!wrapAround || found || (startInd == 0)) {
    // current position in the path is the index of the closest point.
    return minId;
  }
  for (int trialId = 0; trialId <= startInd; trialId++) {
    double thisDist = path_utils::safeGetDistance(tfBuffer, poses[trialId], self);
    if (thisDist < 0.) {
      // something went wrong.
      RCLCPP_ERROR(logger, "INITIALIZATION FAILED due to failed to get distance to path poses.");
      return -1;
    } else if (thisDist < minDistance) {
      minDistance = thisDist;
      minId = trialId;
    } else if (stopAtFirstMin) {
      // distance is increasing. Quit.
      break;
    }
  }
  return minId;
}

}  // namespace path_utils

#endif
