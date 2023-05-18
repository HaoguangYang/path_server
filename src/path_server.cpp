#include "path_server/path_server.hpp"

#include <chrono>
#include <ctime>

namespace path_server {

#ifdef RCLCPP__RCLCPP_HPP_
PathServer::PathServer(rclcpp::Node* node, tf2_ros::Buffer* tfBuffer,
                       const std::string& targetFrame, const std::string& utmFrame,
                       const std::string& localEnuFrame, const std::string& mapFrame,
                       const std::string& odomFrame, const std::string& onNewPath)
#elif defined ROSCPP_ROS_H
PathServer::PathServer(ros::NodeHandle* node, tf2_ros::Buffer* tfBuffer,
                       const std::string& targetFrame, const std::string& utmFrame,
                       const std::string& localEnuFrame, const std::string& mapFrame,
                       const std::string& odomFrame, const std::string& onNewPath)
#endif
    : node_(node),
      tfBuffer_(tfBuffer),
      pathFrame_("earth"),
      targetFrame_(targetFrame),
      onNewPath_(onNewPath),
      lookAheadDist_(100.0),
      lookBehindDist_(0.0),
      isClosedPath_(false),
      doInterp_(true),
      interpStep_(1.0),
      interpretYaw_(true),
      prepareActivePathSeg_(true) {
  // assign header frame ids for path representations
  utmCoords_.header.frame_id = utmFrame;
  localEnuCoords_.header.frame_id = localEnuFrame;
  mapCoords_.header.frame_id = mapFrame;
  odomCoords_.header.frame_id = odomFrame;
  frontPath_.header.frame_id = targetFrame_;
  rearPath_.header.frame_id = targetFrame_;
}

template <typename T, typename Lambda1, typename Lambda2>
std::vector<PoseStamped> PathServer::packPathPoses(const std::vector<T>& coords,
                                                   const std::string& outputFrame,
                                                   Lambda1&& poseMethod, Lambda2&& headingMethod) {
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
   * If global interp, the advancement of current/LA/LB indices
   * per timestep is 1, which means the update step performs no
   * more than two trials. This is a tolerable overhead compared
   * to calculating the local interpolation every step.
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
    auto interp = BezierCurveInterpolator(positions, isClosedPath_);
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
      // interp_angle=
      double* ptr = &yawOut[0];
      Map<VectorXd> angles(ptr, yawOut.size());
      // post-processing: no difference should exceed M_PI
      const double TWO_PI = M_PI + M_PI;
      for (int n = 1; n < angles.size(); n++) {
        if (angles(n) - angles(n - 1) > M_PI)
          angles(n) -= TWO_PI;
        else if (angles(n) - angles(n - 1) < -M_PI)
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
        if (!isClosedPath_) {
          // open path, last element does not turn
          it->pose.orientation = (it - 1)->pose.orientation;
        } else {
          // close the loop
          tf2::Quaternion q = safeGetOrientation(*(it), *(out.begin()));
          it->pose.orientation = tf2::toMsg(q);
        }
        break;
      }
      // Use atan2 to obtain yaw angle, then express in quaternion
      tf2::Quaternion q = safeGetOrientation(*(it), *(it + 1));
      it->pose.orientation = tf2::toMsg(q);
    }
  }
  return out;
}

template <typename T>
std::vector<T> PathServer::getActiveSeg(const std::vector<T>& source, const int& indStart,
                                        const int& indEnd, const bool& isClosed) const {
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
    // going positive direction
    if (ind1 > ind2) {
      target = {source.begin() + ind1, source.end()};
      target.insert(target.end(), source.begin(), source.begin() + 1 + ind2);
    } else {
      target = {source.begin() + ind1, source.begin() + 1 + ind2};
    }
  } else {
    // going negative direction
    if (ind1 < ind2) {
      target = {source.rend() - 1 - ind1, source.rend()};
      target.insert(target.end(), source.rbegin(), source.rend() - ind2);
    } else {
      target = {source.rend() - 1 - ind1, source.rend() - ind2};
    }
  }
  return target;
}

void PathServer::registerPath(const std::string& pathFrame,
                              const std::vector<std::vector<double>>& rawPath) {
  PathCandidate& which = utmCoords_;
  std::vector<PoseStamped> regPath;
  // isolate this variable because it requires re-initialization.
  if (pathFrame == "earth") {
    // First to UTM
    regPath = packPathPoses(
        rawPath, utmCoords_.header.frame_id,
        // Lat/Long to UTM lambda function
        [](const std::vector<double>& latlon) {
          double utmE, utmN;
          std::string utmZoneTmp;
          gps_tools::LLtoUTM(latlon[0], latlon[1], utmN, utmE, utmZoneTmp);
          // std::cout << utmE << " " << utmN << std::endl;
          return std::vector<double>{utmE, utmN};
        },
        // NED heading to ENU yaw lambda function
        [](const double& heading) { return (90.0 - heading) * M_PI / 180.0; });
  } else {
    try {
      which = firstMatchedFrame({utmCoords_, mapCoords_, localEnuCoords_}, pathFrame);
      // directly populate utm, map, or local_enu coords
      regPath = packPathPoses(rawPath, pathFrame);
    } catch (std::runtime_error& e) {
      // Transform here if the path frame is non of the static frames.
      Path tmp_path;
      tmp_path.header.frame_id = pathFrame;
      tmp_path.poses = packPathPoses(rawPath, pathFrame);
      // retries for the initial 10 seconds
      uint8_t retries = 100;
#ifdef RCLCPP__RCLCPP_HPP_
      while (rclcpp::ok())
#elif defined ROSCPP_ROS_H
      while (node_->ok())
#endif
      {
        // most likely this path is w.r.t. some perceived stationary object.
        // tries in the precedence order of map -> local_enu -> utm -> odom
        try {
          which =
              firstTransformable({mapCoords_, localEnuCoords_, utmCoords_, odomCoords_}, pathFrame);
        } catch (std::runtime_error& e) {
          // we got nothing this round. wait 0.1s and try again
#ifdef RCLCPP_INFO_THROTTLE
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
                               "No transform was available. Retrying...");
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
#elif defined ROS_INFO_THROTTLE
          ROS_INFO_THROTTLE(1, "No transform was available. Retrying...");
          ros::Duration(0.1).sleep();
#endif
          retries--;
          if (retries > 0) continue;
#ifdef RCLCPP_FATAL
          RCLCPP_FATAL(
              node_->get_logger(),
              "PROVIDED PATH FRAME IS NOT TRANSFORM-ABLE. THE NODE WILL NOT OUTPUT ANYTHING.");
#elif defined ROS_FATAL
          ROS_FATAL("PROVIDED PATH FRAME IS NOT TRANSFORM-ABLE. THE NODE WILL NOT OUTPUT ANYTHING.");
#endif
          throw std::runtime_error("Path Registration Failed.");
        }
        regPath = safeTransformPoses(which.header.frame_id, tmp_path.poses);
        // we got a valid output.
        if (regPath.size()) break;
      }
    }
  }
  // initial path
  if (!which.poses.size())
    replacePoses(which.poses, regPath);
  else if (onNewPath_ == "append")
    appendPoses(which.poses, regPath);
  else if (onNewPath_ == "replace")
    replacePoses(which.poses, regPath);
  // else if (onNewPath_ == "intersect")
  //   intersectPoses(which.poses, regPath);
  //  TODO: more path insertion approaches
  //  give it a new time
#ifdef RCLCPP__RCLCPP_HPP_
  which.header.stamp = node_->now();
#elif defined ROSCPP_ROS_H
  which.header.stamp = ros::Time::now();
#endif
  which.version = getTimestamp(which.header);
  latestPathVer_ = which.version;
#ifdef RCLCPP__RCLCPP_HPP_
  staticFrameChkTimer_ =
      node_->create_wall_timer(1.0s, std::bind(&PathServer::updateStaticFrameTfAvail, this));
#elif defined ROSCPP_ROS_H
  staticFrameChkTimer_ = node_->createTimer(ros::Duration(1.0), &PathServer::updateStaticFrameTfAvail, this);
#endif
}

bool PathServer::initialize() {
  // path has already been initialized.
  if (currentInd_ >= 0) return true;
  initFrenetXDistance();
  initCurvature();
  std::vector<PoseStamped> transformedPoses;
  // try each representation of the path until the any one of them transforms to the target pose.
  // find the first one that is both available and transformable.
  PathCandidate& which = utmCoords_;
  try {
    which = findFirstThat(
        {utmCoords_, localEnuCoords_, mapCoords_, odomCoords_},
        [&](const auto& item, const std::string& toFrame) {
          if (getTimestamp(item.header) < latestPathVer_ || item.poses.size() <= 0) return false;
          return (safeCanTransform(toFrame, item.header.frame_id, tf2::TimePointZero));
        },
        targetFrame_);
  } catch (std::runtime_error& e) {
    // none of them can. return uninitialized.
#ifdef RCLCPP_WARN_THROTTLE
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *(node_->get_clock()), 1000U,
        "INITIALIZATION FAILED due to unable to transform poses to %s. Retrying...",
        targetFrame_.c_str());
#elif defined ROS_WARN_THROTTLE
    ROS_WARN_THROTTLE(
        1, "INITIALIZATION FAILED due to unable to transform poses to %s. Retrying...",
        targetFrame_.c_str());
#endif
    return false;
  }
  transformedPoses = safeTransformPoses(targetFrame_, which.poses);
  // initializing
  currentInd_ = findClosestInd(transformedPoses, 0, false);

  if (prepareActivePathSeg_) {
    // initialize lookAheadInd, lookBehindInd indices
    lookAheadInd_ = currentInd_ + 1;
    lookBehindInd_ = currentInd_ - 1;
  }
  return true;
}

void PathServer::initFrenetXDistance() {
  std::vector<PoseStamped>& which = utmCoords_.poses;
  try {
    which = firstAvailable(
        {utmCoords_.poses, localEnuCoords_.poses, mapCoords_.poses, odomCoords_.poses});
  } catch (std::runtime_error& e) {
    // paths not initialized
    // TODO: handle exception
    return;
  }
  // TODO: if frenet cumulative distance is already the latest version, skip re-initialization.
  milestone_.clear();
  milestone_.reserve(which.size());
  milestone_.emplace_back(0.0);
  for (auto it = which.cbegin(); it < which.cend() - 1; it++) {
    // std::cout << milestone_.back() << std::endl;
    //  get distance between this and next poses.
    milestone_.emplace_back(milestone_.back() + safeGetDistance(*it, *(it + 1)));
  }
  if (isClosedPath_) {
    milestone_[0] = (milestone_.back() + safeGetDistance(which.back(), which.front()));
  }
  // verify
  if (milestone_[0] < 0) milestone_.clear();
}

void PathServer::initCurvature() {
  std::vector<PoseStamped>& which = utmCoords_.poses;
  try {
    which = firstAvailable(
        {utmCoords_.poses, localEnuCoords_.poses, mapCoords_.poses, odomCoords_.poses});
  } catch (std::runtime_error& e) {
    // paths not initialized
    // TODO: handle exception
    return;
  }
  // TODO: if curvature is already the latest version, skip re-initialization.
  curvature_.clear();
  curvature_.reserve(which.size());
  for (auto it = which.cbegin(); it < which.cend() - 2; it++) {
    // get curvature at this point.
    curvature_.emplace_back(mengerCurvature(*it, *(it + 1), *(it + 2)));
  }
  // std::cout << curvature_.back() << std::endl;
  if (isClosedPath_) {
    curvature_.emplace_back(mengerCurvature(*(which.cend() - 2), which.back(), which.front()));
    curvature_.emplace_back(mengerCurvature(which.back(), which.front(), *(which.cbegin() + 1)));
  } else {
    curvature_.emplace_back(curvature_.back());
    curvature_.emplace_back(0.0);
  }
}

void PathServer::updateLookAheadAndBehindInds() {
  double trialDist;
  int8_t fold = 0;
  /*                     LookAhead / LookBehind distance
   * Search direction: fold = 0 ------>|-------->X fold = 1
   *                    fold = 2  Y<---|<--------  <--------
   *                               ----|-------->X fold = 3 (break)
   */
  if (lookAheadInd_ <= currentInd_) {
    lookAheadInd_ = std::min(currentInd_ + 1, static_cast<int>(milestone_.size()) - 1);
  }
  do {
    // additional break condition when the path is not closed
    if (!isClosedPath_ && lookAheadInd_ >= static_cast<int>(milestone_.size()) - 1) {
      lookAheadInd_ = milestone_.size() - 1;
      break;
    }
    trialDist = getFrenetXDistance(currentInd_, lookAheadInd_);
    if (trialDist >= lookAheadDist_) {
      if (fold == 2) break;  // exactly one index over the distance bounds.
      fold = 1;
      lookAheadInd_--;
    } else {
      if (fold == 1) fold = 2;
      lookAheadInd_++;
    }
    // std::cout << "stuck in second while loop:" << lookAheadInd_ << "," << currentInd_ << ","
    //           << trialDist << std::endl;
  } while (lookAheadInd_ != currentInd_);

  // lookAheadInd_: first index that exceeds lookAheadDistance from currentInd_.
  fold = 0;
  if (lookBehindInd_ >= currentInd_) {
    lookBehindInd_ = std::max(currentInd_ - 1, 0);
  }
  do {
    // additional break condition when the path is not closed
    if (!isClosedPath_ && lookBehindInd_ <= 0) {
      lookBehindInd_ = 0;
      break;
    }
    trialDist = getFrenetXDistance(lookBehindInd_, currentInd_);
    if (trialDist >= lookBehindDist_) {
      if (fold == 2) break;  // exactly one index over the distance bounds.
      fold = 1;
      lookBehindInd_++;
    } else {
      if (fold == 1) fold = 2;
      lookBehindInd_--;
    }
    // std::cout << "stuck in second while loop:" << lookBehindInd_ << "," << currentInd_ << ","
    //           << trialDist << std::endl;
  } while (lookBehindInd_ != currentInd_);
  // lookBehindInd_: first index that exceeds lookBehindDistance from currentInd_.
}

#ifdef RCLCPP__RCLCPP_HPP_
void PathServer::updateStaticFrameTfAvail()
#elif defined ROSCPP_ROS_H
void PathServer::updateStaticFrameTfAvail(const ros::TimerEvent& event)
#endif
{
  // if has a static frame but its path representation does not exist, create one.
  // from local ENU or map to UTM
  syncPathCandidate({localEnuCoords_, mapCoords_}, utmCoords_);
  // from UTM or local ENU to Map
  syncPathCandidate({utmCoords_, localEnuCoords_}, mapCoords_);

  // odom frame coordinates will NOT propagate outwards, but will ONLY propages
  // inwards (convert from X to odom frame), since it is NOT a strictly static frame.
  // always update odom coords copy because odom frame can drift significantly.
  odomCoords_.version = 0;
  syncPathCandidate({utmCoords_, localEnuCoords_, mapCoords_}, odomCoords_);
}

/* publish path in target_frame, extracting the segment within
 * [lookBehind, lookAhead]
 * */
bool PathServer::updateStep() {
  // TODO: if path has not been initialized yet: clear and return empty path.
  if (currentInd_ < 0) {
    return false;
  }
  // std::vector<PoseStamped> *which;
  PathCandidate& which = utmCoords_;
  try {
    which =
        firstTransformable({utmCoords_, localEnuCoords_, mapCoords_, odomCoords_}, targetFrame_);
  } catch (std::runtime_error& e) {
#ifdef RCLCPP_ERROR_THROTTLE
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *(node_->get_clock()), 1000,
                          "Cannot transform frames or the poses are not initialized.");
#elif defined ROS_ERROR_THROTTLE
    ROS_ERROR_THROTTLE(1, "Cannot transform frames or the poses are not initialized.");
#endif
    // TODO: cannot transform: clear and return empty path.
    return false;
  }
  // start from the current index,
  // and determine when to increment the current index.
  int newClosestInd = findClosestInd(which.poses, currentInd_, true, isClosedPath_);
  if (newClosestInd < 0) {
    // TODO: Find Closest Index failed. clear and return empty path.
    return false;
  }

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
        safeTransformPoses(targetFrame_, std::vector<PoseStamped>{which.poses[currentInd_]});
    frontPath_.offset_poses = frontPath_.poses;
    frontPath_.offset_left = {0.};
    frontPath_.curvature = {curvature_[currentInd_]};
    rearPath_ = frontPath_;
    return true;
  }

  /* distance to trialInd = trialDist.
   * trialInd is the index of point that is closest
   * to self from curInd and on.
   * */
  int delta = newClosestInd - currentInd_;
  const int half = which.poses.size() / 2;
  if (delta < -half) {
    if (!isClosedPath_) {
#ifdef RCLCPP_FATAL
      RCLCPP_FATAL(node_->get_logger(), "LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
#elif defined ROS_FATAL
      ROS_FATAL("LOOP CLOSURE OCCURED IN A NON-CLOSED PATH!");
#endif
    } else {
      lapCount_++;
      delta += which.poses.size();
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
      delta -= which.poses.size();
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
      targetFrame_, getActiveSeg(which.poses, currentInd_, lookAheadInd_, isClosedPath_));
#ifdef RCLCPP__RCLCPP_HPP_
  rearPath_.header.stamp = node_->now();
#elif defined ROSCPP_ROS_H
  rearPath_.header.stamp = ros::Time::now();
#endif
  rearPath_.header.frame_id = targetFrame_;
  rearPath_.poses = safeTransformPoses(
      targetFrame_, getActiveSeg(which.poses, currentInd_, lookBehindInd_, isClosedPath_));
  frontPath_.offset_poses = frontPath_.poses;
  rearPath_.offset_poses = rearPath_.poses;

  // Transform-invariant fields
  // update curvature
  // service rear curvature array with index delta
  // moving forward, feed rear curvature with the closest front curvature
  for (int n = 0; n < delta; n++) {
    rearPath_.curvature.push_front(frontPath_.curvature.size() ? frontPath_.curvature.front() : 0.);
    if (frontPath_.curvature.size() > 1) frontPath_.curvature.pop_front();
  }
  // moving backward, drop the closest rear curvature
  for (int n = 0; (n < -delta) && (rearPath_.curvature.size() >= 1); n++) {
    rearPath_.curvature.pop_front();
  }
  // resize rear curvature to make alignment
  rearPath_.curvature.resize(rearPath_.offset_poses.size(),
                             rearPath_.curvature.size() ? rearPath_.curvature.back() : 0.);
  // service front curvature array
  auto curv_tmp = getActiveSeg(curvature_, currentInd_, lookAheadInd_, isClosedPath_);
  frontPath_.curvature = {curv_tmp.begin(), curv_tmp.end()};

  // service rear offset array with index delta
  // moving forward, feed rear offset with the closest front offset
  for (int n = 0; n < delta; n++) {
    rearPath_.offset_left.push_front(frontPath_.offset_left.size() ? frontPath_.offset_left.front()
                                                                   : 0.);
    if (frontPath_.offset_left.size() > 1) frontPath_.offset_left.pop_front();
  }
  // moving backward, drop the closest rear offset
  for (int n = 0; (n < -delta) && (rearPath_.offset_left.size() >= 1); n++) {
    rearPath_.offset_left.pop_front();
  }
  // resize rear offset to make alignment
  rearPath_.offset_left.resize(rearPath_.offset_poses.size(),
                               rearPath_.offset_left.size() ? rearPath_.offset_left.back() : 0.);
  // service front offset array
  if (doOffset_) {
    // if offset, offset the path per request.
    // offset message format:
    // header
    // float32 lateral_offset (m, global frenet, +x is left.)
    // float32 effective_from_distance (m, global frenet)
    // float32 effective_until_distance (m, global frenet)
    // ease in the path offset from the received point.
    frontPath_.offset_left.clear();
    // Follow a ease-in -- hold -- ease-out pattern based on
    // odomMark_, odomSet_, odomLift, and odomClear_
    for (size_t n = 0; n < frontPath_.offset_poses.size(); n++) {
      double odomAtPoint = odometer_ + getFrenetXDistance(currentInd_, currentInd_ + n);
      float nodeOffset = 0.0;
      if (odomAtPoint < pathOffsetOdomMark_)
        nodeOffset = existingOffset_;
      else if (odomAtPoint < pathOffsetOdomSet_) {
        float ratio = (odomAtPoint - pathOffsetOdomMark_) / pathOffsetEaseInDist_;
        nodeOffset = ratio * pathOffsetToLeft_ + (1. - ratio) * existingOffset_;
      } else if (odomAtPoint < pathOffsetOdomLift_)
        nodeOffset = pathOffsetToLeft_;
      else if (odomAtPoint < pathOffsetOdomClear_)
        nodeOffset = pathOffsetToLeft_ * (pathOffsetOdomClear_ - odomAtPoint) /
                     (pathOffsetOdomClear_ - pathOffsetOdomLift_);
      // else: nodeOffset = 0.0
      frontPath_.offset_left.emplace_back(nodeOffset);
    }

    /*
    // TODO: I doubt this is necessary. The heading change may be minimal.
    // In certain cases (e.g. a holonomic robot), we may not want the target heading to change at
    // all.
    double hdgCorrection = std::atan2(pathOffsetToLeft_, pathOffsetEaseInDist_);
    int i = 0;
    for (auto& pose : frontPath_.offsetPoses) {
      // add heading correction to the quaternion.
      if (frontPath_.offset_left[i] != pathOffsetToLeft_) {
        tf2::Quaternion q_corrected;
        q_corrected.setRPY(0., 0., hdgCorrection + q.getAngle());
        pose.pose.orientation = tf2::toMsg((q_corrected * q).normalize());
      }
      i++;
    }
    */

    std::transform(frontPath_.offset_left.cbegin(), frontPath_.offset_left.cend(),
                   frontPath_.offset_poses.cbegin(), frontPath_.offset_poses.begin(),
                   [&](const float& offset, const PoseStamped& pose) {
                     PoseStamped ret = pose;
                     // offset pose to its left by offsetArray[i]. Use quaternion of the pose.
                     tf2::Quaternion q;
                     tf2::fromMsg(pose.pose.orientation, q);
                     const tf2::Vector3 offsetVect =
                         tf2::quatRotate(q, tf2::Vector3(0., offset, 0.));
                     ret.pose.position.x += offsetVect.x();
                     ret.pose.position.y += offsetVect.y();
                     ret.pose.position.z += offsetVect.z();
                     return ret;
                   });
    // treat curvature array
    std::transform(frontPath_.offset_left.cbegin(), frontPath_.offset_left.cend(),
                   frontPath_.curvature.cbegin(), frontPath_.curvature.begin(),
                   [](const float& offset, const float& curvature) {
                     return (fabs(curvature) <= FLT_EPSILON) ? 0. : 1. / (1. / curvature - offset);
                   });
    // Reset path offset flag when the offset segment has passed
    if (odometer_ > pathOffsetOdomClear_) {
      doOffset_ = false;
    }
  } else {
    frontPath_.offset_left.resize(frontPath_.offset_poses.size(), 0.);
  }
  return true;
}

/* Decommissioned functions */

/*
// Decommissioned this part due to high branching overhead.
// (given that memory copy is unavoidable, we don't care about a full copy of the path segment)

// This API maps to getActiveSeg.
template <typename T>
void PathServer::updateActiveSegFull(std::deque<T>& target, const std::vector<T>& source,
                                     const int& indStart, const int& indEnd, const bool& isClosed);

template <typename T>
void PathServer::updateActiveSegIncr(std::deque<T>& target, const std::vector<T>& source,
                                     const int& oldIndStart, const int& newIndStart,
                                     const int& oldIndEnd, const int& newIndEnd,
                                     const bool& isClosed) {
  // one element
  if (newIndStart == newIndEnd) {
    target = {source[newIndStart]};
    return;
  }

  // check bounds when the path is not closed.
  if (!isClosed) {
    if (oldIndStart >= static_cast<int>(source.size()) || oldIndStart < 0 ||
        newIndStart >= static_cast<int>(source.size()) || newIndStart < 0 ||
        oldIndEnd >= static_cast<int>(source.size()) || oldIndEnd < 0 ||
        newIndEnd >= static_cast<int>(source.size()) || newIndEnd < 0) {
      target.clear();
      return;
    }
  }

  int oldSize = oldIndEnd - oldIndStart;
  // empty start or single element or size mismatch.
  if (oldSize == 0 || target.size() != static_cast<size_t>(abs(oldSize) + 1)) {
    updateActiveSegFull(target, source, newIndStart, newIndEnd, isClosed);
    return;
  }

  int ind1 = oldIndStart;
  int ind2 = newIndStart;
  int ind3 = oldIndEnd;
  int ind4 = newIndEnd;
  // preprocess indices
  if (isClosed) {
    ind1 %= source.size();
    if (ind1 < 0) ind1 += source.size();
    ind2 %= source.size();
    if (ind2 < 0) ind2 += source.size();
    ind3 %= source.size();
    if (ind3 < 0) ind3 += source.size();
    ind4 %= source.size();
    if (ind4 < 0) ind4 += source.size();
  }

  // unroll delta values
  int startDelta = ind2 - ind1;
  int endDelta = ind4 - ind3;
  if (isClosed) {
    const int half = static_cast<int>(source.size() / 2);
    if (startDelta > half)
      startDelta -= source.size();
    else if (startDelta < -half)
      startDelta += source.size();
    if (endDelta > half)
      endDelta -= source.size();
    else if (endDelta < -half)
      endDelta += source.size();
  }

  // when to use full update:
  // segment direction has changed. This is not allowed.
  // startDelta is more than current size, at the current direction, or
  // endDelta is more than current size, at the opposite direction.
  // Then all existing contents will be erased.
  if ((std::signbit(oldSize - startDelta + endDelta) ^ std::signbit(oldSize)) ||
      ((startDelta > oldSize) ^ (oldSize < 0)) || ((endDelta < -oldSize) ^ (oldSize < 0))) {
    updateActiveSegFull(target, source, newIndStart, newIndEnd, isClosed);
    return;
  }

  // handle start delta
  // erase start
  if (std::signbit(startDelta) == std::signbit(oldSize)) {
    target.erase(target.begin(), target.begin() + abs(startDelta));
  } else {
    // insert start
    if (startDelta < 0) {
      // oldSize > 0. Going positive direction
      if (ind2 > ind1) {
        target.insert(target.begin(), source.begin(), source.begin() + ind1);
        target.insert(target.begin(), source.begin() + ind2, source.end());
      } else {
        target.insert(target.begin(), source.begin() + ind2, source.begin() + ind1);
      }
    } else {
      // oldSize < 0. Going inverse direction
      if (ind2 < ind1) {
        target.insert(target.begin(), source.rbegin(), source.rend() - 1 - ind1);
        target.insert(target.begin(), source.rend() - 1 - ind2, source.rend());
      } else {
        target.insert(target.begin(), source.rend() - 1 - ind2, source.rend() - 1 - ind1);
      }
    }
  }
  // handle end data
  // erase end
  if (std::signbit(endDelta) ^ std::signbit(oldSize)) {
    target.erase(target.end() - abs(endDelta), target.end());
  } else {
    // insert end
    if (endDelta < 0) {
      // oldSize < 0 as well. Going inverse direction
      if (ind4 > ind3) {
        target.insert(target.end(), source.rend() - ind3, source.rend());
        target.insert(target.end(), source.rbegin(), source.rend() - ind4);
      } else {
        target.insert(target.end(), source.rend() - ind3, source.rend() - ind4);
      }
    } else {
      // oldSize > 0 as well. Going positive direction
      if (ind4 < ind3) {
        target.insert(target.end(), source.begin() + 1 + ind3, source.end());
        target.insert(target.end(), source.begin(), source.begin() + 1 + ind4);
      } else {
        target.insert(target.end(), source.begin() + 1 + ind3, source.begin() + 1 + ind4);
      }
    }
  }
}
*/

}  // namespace path_server
