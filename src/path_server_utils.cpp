#include "path_server/path_server.hpp"

namespace path_server {

double PathServer::getFrenetXDistance(const int& ind1, const int& ind2) {
  if (!milestone_.size()) {
    initFrenetXDistance();
    // initialization failed.
    if (!milestone_.size()) return 0.0;
  }
  if (!isClosedPath_) {
    // direct subtraction from the cumulative distance vector.
    return milestone_[std::clamp(ind2, 0, static_cast<int>(milestone_.size()) - 1)] -
           milestone_[std::clamp(ind1, 0, static_cast<int>(milestone_.size()) - 1)];
  } else {
    /* we are running a closed path. Need to consider crossing the starting point.
     * Hence, we allow the indices to exceed poses.size(), indicating running along
     * the specified direction.
     * */
    int laps = ind2 / static_cast<int>(milestone_.size()) - ind1 / static_cast<int>(milestone_.size());
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
    return milestone_[0] * laps + (rem2 ? milestone_[rem2] : 0.0) - (rem1 ? milestone_[rem1] : 0.0);
  }
}

int PathServer::findClosestInd(const std::vector<PoseStamped>& poses, const int& startInd,
                               bool stopAtFirstMin, bool wrapAround) const {
  int minId = startInd;
  double minDistance = DBL_MAX;
  const PoseStamped self = [&]() {
    PoseStamped ret;
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
  bool found = false;
  // find next closest point from startInd towards the end
  for (int trialId = startInd; trialId < static_cast<int>(poses.size()); trialId++) {
    double thisDist = safeGetDistance(poses[trialId], self);
    if (thisDist < 0.) {
      // something went wrong.
#ifdef RCLCPP_ERROR
      RCLCPP_ERROR(node_->get_logger(),
                   "INITIALIZATION FAILED due to failed to get distance to path poses.");
#elif defined ROS_ERROR
      ROS_ERROR("INITIALIZATION FAILED due to failed to get distance to path poses.");
#endif
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
    double thisDist = safeGetDistance(poses[trialId], self);
    if (thisDist < 0.) {
      // something went wrong.
#ifdef RCLCPP_ERROR
      RCLCPP_ERROR(node_->get_logger(),
                   "INITIALIZATION FAILED due to failed to get distance to path poses.");
#elif defined ROS_ERROR
      ROS_ERROR("INITIALIZATION FAILED due to failed to get distance to path poses.");
#endif
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

std::tuple<double, double, double> PathServer::safeVector(const PoseStamped& pose1,
                                                          const PoseStamped& pose2) const {
  double dx, dy, dz;
  if (pose1.header.frame_id != pose2.header.frame_id) {
    if (!safeCanTransform(pose1.header.frame_id, pose2.header.frame_id, tf2::TimePointZero)) {
      return std::make_tuple(NAN, NAN, NAN);
    }
    PoseStamped pose_alt = tfBuffer_->transform(pose2, pose1.header.frame_id);
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

Pose PathServer::safeProjection(const PoseStamped& pose1, const PoseStamped& pose2) const {
  Pose res;
  PoseStamped pose_alt = pose2;
  if (pose1.header.frame_id != pose2.header.frame_id) {
    if (!safeCanTransform(pose1.header.frame_id, pose2.header.frame_id, tf2::TimePointZero)) {
      res.position.x = NAN;
      res.position.y = NAN;
      res.position.z = NAN;
      res.orientation.w = NAN;
      res.orientation.x = 0.0;
      res.orientation.y = 0.0;
      res.orientation.z = 0.0;
      return res;
    }
    PoseStamped pose_alt = tfBuffer_->transform(pose2, pose1.header.frame_id);
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

double PathServer::safeGetDistance(const PoseStamped& pose1, const PoseStamped& pose2) const {
  auto vect = safeVector(pose1, pose2);
  if (isnan(std::get<0>(vect)) || isnan(std::get<1>(vect)) || isnan(std::get<2>(vect))) return -1;
  return std::sqrt(std::get<0>(vect) * std::get<0>(vect) + std::get<1>(vect) * std::get<1>(vect) +
                   std::get<2>(vect) * std::get<2>(vect));
}

tf2::Quaternion PathServer::safeGetOrientation(const PoseStamped& pose1, const PoseStamped& pose2) const {
  tf2::Quaternion q;
  auto vect = safeVector(pose1, pose2);
  if (isnan(std::get<0>(vect)) || isnan(std::get<1>(vect)) || isnan(std::get<2>(vect))) {
    // throw std::runtime_error("Vector could not be obtained between path poses.");
    return q.getIdentity();
  }
  double yaw = std::atan2(std::get<1>(vect), std::get<0>(vect));
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

}  // namespace path_server
