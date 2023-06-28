#ifndef _PATH_SERVER__PATH_SERVER_HPP_
#define _PATH_SERVER__PATH_SERVER_HPP_

#include <stdarg.h>

#include <chrono>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <memory>
#include <stdexcept>

#if __has_include("rclcpp/rclcpp.hpp")
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#elif __has_include("ros/ros.h")
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"
#endif

#include "bezier_interpolation.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/buffer_core.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "GeographicLib/UTMUPS.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace Eigen;
#ifdef RCLCPP__RCLCPP_HPP_
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using std_msgs::msg::Header;
#elif defined ROSCPP_ROS_H
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using nav_msgs::Path;
using std_msgs::Header;
#endif

#ifdef ROSCPP_ROS_H
namespace tf2 {
  typedef ros::Time TimePoint;
  const ros::Time TimePointZero = ros::Time(0);
}
#endif

namespace path_server {
class PathServer {
 public:
#ifdef RCLCPP__RCLCPP_HPP_
  explicit PathServer(rclcpp::Node* node, tf2_ros::Buffer* tfBuffer, const std::string& targetFrame,
                      const std::string& utmFrame, const std::string& localEnuFrame,
                      const std::string& mapFrame, const std::string& odomFrame,
                      const std::string& onNewPath, const double& utmScale = 1.0);
#elif defined ROSCPP_ROS_H
  explicit PathServer(ros::NodeHandle* node, tf2_ros::Buffer* tfBuffer,
                      const std::string& targetFrame, const std::string& utmFrame,
                      const std::string& localEnuFrame, const std::string& mapFrame,
                      const std::string& odomFrame, const std::string& onNewPath,
                      const double& utmScale = 1.0);
#endif

  ~PathServer() = default;

  void setPathAttr(const double& lookAheadDist, const double& lookBehindDist,
                   const bool& isClosedPath, const bool& doInterp, const double& interpStep,
                   const bool& interpretYaw, const bool& prepareActivePathSeg) {
    lookAheadDist_ = lookAheadDist;
    lookBehindDist_ = lookBehindDist;
    isClosedPath_ = isClosedPath;
    interpretYaw_ = interpretYaw;
    doInterp_ = doInterp;
    interpStep_ = interpStep;
    prepareActivePathSeg_ = prepareActivePathSeg;
  };

  void setOffsetRateLimit(const float& rateLim) { pathOffsetEaseInRate_ = rateLim; };

  /**
   * @brief convert x_file_, y_file_, yaw_file_ to a static standard frame
   * (map/utm/local_enu). This step does not require transform tree.
   *
   * @param pathFrame
   * @param rawPath
   */
  void registerPath(const std::string& pathFrame, const std::vector<std::vector<double>>& rawPath);

  /**
   * @brief find current closest point on the path coordinates,
   * and output the index as a class variable.
   *
   * @return true
   * @return false
   */
  bool initialize();

  bool updateStep();

  /**
   * @brief Sync paths in different static reference frames.
   * The availability of these frames can change over time.
   */
#ifdef RCLCPP__RCLCPP_HPP_
  void updateStaticFrameTfAvail();
#elif defined ROSCPP_ROS_H
  void updateStaticFrameTfAvail(const ros::TimerEvent&);
#endif

  void offsetPath(const float& from, const float& to, const float& leftingAmount);

  void clearOffset() { doOffset_ = false; };

  Path getFrontPath() const {
    Path out;
    out.header = frontPath_.header;
    out.poses = frontPath_.poses;
    return out;
  };

  Path getRearPath() const {
    Path out;
    out.header = rearPath_.header;
    out.poses = rearPath_.poses;
    return out;
  };

  Path getFrontOffsetPath() const {
    Path out;
    out.header = frontPath_.header;
    out.poses = frontPath_.offset_poses;
    return out;
  };

  Path getRearOffsetPath() const {
    Path out;
    out.header = rearPath_.header;
    out.poses = rearPath_.offset_poses;
    return out;
  };

  std::vector<float> getFrontLookAheadArray() const {
    return frontPath_.look_ahead;
  }

  std::vector<float> getFrontCurvature() const {
    return {frontPath_.curvature.cbegin(), frontPath_.curvature.cend()};
  }

  std::vector<float> getRearCurvature() const {
    return {rearPath_.curvature.cbegin(), rearPath_.curvature.cend()};
  }

  std::vector<float> getFrontOffsetArray() const {
    return {frontPath_.offset_left.cbegin(), frontPath_.offset_left.cend()};
  }

  std::vector<float> getRearOffsetArray() const {
    return {rearPath_.offset_left.cbegin(), rearPath_.offset_left.cend()};
  }

  const float& getCurrentOffset() const { return frontPath_.offset_left.front(); }

  const int& getCurrentInd() const { return currentInd_; };

  const bool& isClosedPath() const { return isClosedPath_; };

  double getPathCumulativeDist() const {
    return isClosedPath_ ? milestone_.front() : milestone_.back();
  };

  const PoseStamped& getClosestPose() const { return frontPath_.offset_poses[0]; };

  std::tuple<double, double, double> safeVector(const PoseStamped& pose1,
                                                const PoseStamped& pose2) const;

  // observe pose2 from pose1
  Pose safeProjection(const PoseStamped& pose1, const PoseStamped& pose2) const;

  /**
   * @brief gets distance between two poses with frame_id handling
   *
   * @param pose1
   * @param pose2
   * @return double
   */
  double safeGetDistance(const PoseStamped& pose1, const PoseStamped& pose2) const;

  /**
   * @brief direction to pose 2 from pose1, measured in pose1 frame.
   *
   * @param pose1
   * @param pose2
   * @return tf2::Quaternion
   */
  tf2::Quaternion safeGetOrientation(const PoseStamped& pose1, const PoseStamped& pose2) const;

  /**
   * @brief transform path.poses using tf2
   *
   * @param outFrame
   * @param inPoses
   * @return std::vector<PoseStamped>
   */
  std::vector<PoseStamped> safeTransformPoses(const std::string& outFrame,
                                              const std::vector<PoseStamped>& inPoses) const {
    std::vector<PoseStamped> outPoses;
    if (safeCanTransform(outFrame, inPoses[0].header.frame_id, tf2::TimePointZero)) {
      const auto tf =
          tfBuffer_->lookupTransform(outFrame, inPoses[0].header.frame_id, tf2::TimePointZero);
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

 private:
  typedef struct PathCandidate {
    const std::string frame_id;
    unsigned long version;
    std::vector<PoseStamped> poses;

    PathCandidate(const std::string& frame_id) : frame_id(frame_id) {}
  } PathCandidate;

  typedef struct SlidingPathWithProperty {
    Header header;
    std::vector<PoseStamped> poses;
    std::vector<float> look_ahead;
    std::deque<float> curvature;
    std::deque<float> offset_left;
    std::vector<PoseStamped> offset_poses;
  } SlidingPathWithProperty;

  /**
   * @brief directly pack raw vector of poses (from file) into path.poses
   *
   * @param coords
   * @param outputFrame
   * @return template <typename T>
   */
  template <typename T>
  std::vector<PoseStamped> packPathPoses(const std::vector<T>& coords,
                                         const std::string& outputFrame) {
    return packPathPoses(
        coords, outputFrame, [](const T& xp) { return xp; }, [](const auto& xr) { return xr; });
  }

  /**
   * @brief pack raw vector of poses (from file) into path.poses with
   * custom transformation functions
   *
   * @tparam T
   * @tparam Lambda1
   * @tparam Lambda2
   * @param coords
   * @param outputFrame
   * @param poseMethod
   * @param headingMethod
   * @return std::vector<PoseStamped>
   */
  template <typename T, typename Lambda1, typename Lambda2>
  std::vector<PoseStamped> packPathPoses(const std::vector<T>& coords,
                                         const std::string& outputFrame, Lambda1&& poseMethod,
                                         Lambda2&& headingMethod);

  /**
   * @brief This method completely rebuilds the path segment.
   *
   * @param source
   * @param indStart
   * @param indEnd
   * @return std::vector<T>
   */
  template <typename T>
  std::vector<T> getActiveSeg(const std::vector<T>& source, const int& indStart, const int& indEnd,
                              const bool& isClosed) const;

  // a power tool to select the first member within a list that has a user-defined property
  template <typename Lambda, typename T, typename... Args>
  T findFirstThat(const std::initializer_list<T>& candidates, const Lambda&& method,
                  const Args&... args) const {
    for (const auto& item : candidates) {
      if (method(item, args...)) {
        return item;
      }
    }
    throw std::runtime_error("None of the provided candidates meet the conditions.");
  }

  template <typename T>
  T firstAvailable(const std::initializer_list<T>& candidates) const {
    return findFirstThat(candidates, [](const T& item) { return (item->size() > 0); });
  }

  static Path makePath(SlidingPathWithProperty& in) {
    Path out;
    out.header = in.header;
    out.poses = in.poses;
    return out;
  }

  /**
   * @brief A path registration method
   *
   * @param target
   * @param input
   */
  void appendPoses(std::vector<PoseStamped>& target, const std::vector<PoseStamped>& input);

  void replacePoses(std::vector<PoseStamped>& target, const std::vector<PoseStamped>& input);

  // void intersectPoses(std::vector<PoseStamped> &target, const std::vector<PoseStamped> &input);

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

  /**
   * @brief initializer for milestone_ vector. Used for frenet frame distance
   * calculation.
   */
  void initFrenetXDistance();

  /**
   * @brief initializer for curvature_ vector. Used to bake curvature of interpolated paths.
   */
  void initCurvature();

  static float mengerCurvature(const PoseStamped& p1, const PoseStamped& p2,
                               const PoseStamped& p3) {
    // TODO: handle exception
    assert(p1.header.frame_id == p2.header.frame_id && p2.header.frame_id == p3.header.frame_id);
    double TrigA =
        (p2.pose.position.x - p1.pose.position.x) * (p3.pose.position.y - p1.pose.position.y) -
        (p2.pose.position.y - p1.pose.position.y) * (p3.pose.position.x - p1.pose.position.x);
    // left turn has positive curvature
    return static_cast<float>(
        4. * TrigA /
        (1.e-12 + std::sqrt((std::pow(p2.pose.position.x - p1.pose.position.x, 2) +
                             std::pow(p2.pose.position.y - p1.pose.position.y, 2)) *
                            (std::pow(p3.pose.position.x - p2.pose.position.x, 2) +
                             std::pow(p3.pose.position.y - p2.pose.position.y, 2)) *
                            (std::pow(p3.pose.position.x - p1.pose.position.x, 2) +
                             std::pow(p3.pose.position.y - p1.pose.position.y, 2)))));
  };

  /**
   * @brief Get the Frenet X Distance object.
   * This function returns the frenet-frame distance between two indices on the
   * stored path (using milestone_). The distance is directional, from ind1 to ind2.
   * If the path is closed, this function allows the two numbers to exceed the
   * size limit of the stored path. The corresponding point is found using modulo.
   *
   * @param ind1
   * @param ind2
   * @return double
   */
  double getFrenetXDistance(const int& ind1, const int& ind2);

  /**
   * @brief This function finds the index on the path that is closest to the current
   * position. The returned index is in the range of [0, poses.size()]
   *
   * @param poses
   * @param startInd
   * @param stopAtFirstMin
   * @param wrapAround
   * @return int
   */
  int findClosestInd(const std::vector<PoseStamped>& poses, const int& startInd,
                     bool stopAtFirstMin = true, bool wrapAround = false) const;

  /**
   * @brief update indices where the path is cut off at front and rear directions.
   * The resultant indices are different from the current (nearest) path pose index,
   * unless it is one of the starting or ending poses.
   * The resultant indices follow path directions when seen at currentInd_, i.e.
   * if the path is closed, the lookForwardInd_ may exceed path size, and
   * the lookBehindInd_ may be negative. These indices are unwrapped values.
   */
  void updateLookAheadAndBehindInds();

  bool safeCanTransform(const std::string& to, const std::string& from,
                        const tf2::TimePoint& time = tf2::TimePointZero) const {
    if (!(this->tfBuffer_->_frameExists(to) && tfBuffer_->_frameExists(from))) return false;
    return (this->tfBuffer_->canTransform(to, from, time));
  }

#ifdef RCLCPP__RCLCPP_HPP_
  rclcpp::Node* node_;
#elif defined ROSCPP_ROS_H
  ros::NodeHandle* node_;
#endif
  tf2_ros::Buffer* tfBuffer_;

  /**
   * @brief User-specified expression of the raw path and the targeted transformation.
   */
  std::string pathFrame_, targetFrame_;

  std::string onNewPath_;
  double lookAheadDist_, lookBehindDist_;
  bool isClosedPath_, doInterp_;
  double interpStep_;
  bool interpretYaw_, prepareActivePathSeg_;

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
  bool utmScaled_ = false;
  double utmScale_ = 1.0;

  unsigned long latestPathVer_ = 0;
  std::vector<double> milestone_;
  std::vector<float> curvature_;
#ifdef RCLCPP__RCLCPP_HPP_
  rclcpp::TimerBase::SharedPtr staticFrameChkTimer_{nullptr};
#elif defined ROSCPP_ROS_H
  ros::Timer staticFrameChkTimer_;
#endif

  int currentInd_ = -1;
  int lookAheadInd_ = 0;
  int lookBehindInd_ = 0;
  int lapCount_ = 0;
  double odometer_ = 0.0;

  // limit on dist_lateral_travelled/dist_forward_travelled. This controls how abrupt the change is.
  float pathOffsetEaseInRate_ = 0.1;
  // offset control points
  bool doOffset_ = false;
  float pathOffsetEaseInDist_ = 50.0;
  double pathOffsetOdomMark_, pathOffsetOdomSet_, pathOffsetOdomLift_, pathOffsetOdomClear_;
  // offset value when the command is received, and the commanded value. Used to calculate ease-in.
  float existingOffset_ = 0.0;
  float pathOffsetToLeft_ = 0.0;

  // output buffer
  SlidingPathWithProperty frontPath_, rearPath_;

};  // class

}  // namespace path_server

#endif  // _PATH_SERVER__PATH_SERVER_HPP_
