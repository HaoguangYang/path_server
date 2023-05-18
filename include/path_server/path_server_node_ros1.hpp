#ifndef _PATH_SERVER__PATH_SERVER_NODE_HPP_
#define _PATH_SERVER__PATH_SERVER_NODE_HPP_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "path_server/path_server.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"

using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using nav_msgs::Path;
using std_msgs::Bool;
using std_msgs::Float32MultiArray;

namespace path_server {
class PathServerNode {
 public:
  explicit PathServerNode(ros::NodeHandle* nh);

  ~PathServerNode() {
    delete pathServer_;
  };

  /**
   * @brief Register a csv path file with the path server instance.
   *
   * @param ps
   * @param pathFileName
   * @param pathFrame
   * @return true
   * @return false
   */
  bool initialize(PathServer* ps, const std::string& pathFileName, const std::string& pathFrame);

  void run();

 private:
  std::vector<std::vector<double>> readPathFromCsv(const std::string& pathFileName);

  void pubErrMsg(const std::string& description, const int8_t& level, const int8_t& lifetime);

  ros::NodeHandle* node_;

  /**
   * @brief path server instances
   */
  PathServer *pathServer_;

  std::unique_ptr<ros::Rate> pathPubTimer_;
  double lookAheadDist_, lookBehindDist_, interpStep_;
  bool isClosedPath_, interpretYaw_, doInterp_, doOffset_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::string frontPathTopic_, rearPathTopic_;

  /**
   * @brief the center, left and right paths all support hot-swapping.
   * set the new parameter for *Alt* path file names, and toggle the path switching boolean.
   */
  std::string pathFrame_, pathFileName_;

  float minLeftwardsOffset_ = 0., maxLeftwardsOffset_ = 0.;

  ros::Publisher offsetRangePub_, pathCurvaturePub_, offsetDistPub_;

  // rclcpp::Publisher<ErrorReport>::SharedPtr errPub_;

  ros::Publisher frontPathPub_, rearPathPub_;

};  // class

}  // namespace path_server

#endif  // _PATH_SERVER__PATH_SERVER_HPP_
