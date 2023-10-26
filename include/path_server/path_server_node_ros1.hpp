/**
 * @file path_server_node_ros1.hpp
 * @author Haoguang Yang (yang1510@purdue.edu)
 * @brief Node class for the path server package. It orchestrates multiple path server instances.
 * @version 0.1.1
 * @date 2023-03-12
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

#ifndef _PATH_SERVER__PATH_SERVER_NODE_HPP_
#define _PATH_SERVER__PATH_SERVER_NODE_HPP_

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "path_server/path_server.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/transform_listener.h"

using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using std_msgs::Bool;

namespace planning {
class PathServerNode {
 public:
  explicit PathServerNode(ros::NodeHandle* nh);

  ~PathServerNode() {
    delete pathServer_;
    delete leftBoundaryServer_;
    delete rightBoundaryServer_;
    delete altPathServer_;
    delete leftAltBoundaryServer_;
    delete rightAltBoundaryServer_;
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
  bool registerPath(PathServer* ps, const std::string& pathFileName, const bool& isPathClosed,
                  const std::string& pathFrame);

  void run();

 private:
  void pubErrMsg(const std::string& description, const int8_t& level, const int8_t& lifetime);

  bool swapActivePathPrecond();

  void swapActivePath();

  ros::NodeHandle* node_;

  /**
   * @brief path server instances
   */
  PathServer *pathServer_, *altPathServer_, *leftBoundaryServer_, *rightBoundaryServer_,
      *leftAltBoundaryServer_, *rightAltBoundaryServer_;

  std::unique_ptr<PeerHandle> pathHandle_, altPathHandle_;

  bool isClosedAltPath_, interpretYaw_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<ros::Rate> updateTimer_;

  /**
   * @brief the center, left and right paths all support hot-swapping.
   * set the new parameter for *Alt* path file names, and toggle the path switching boolean.
   */
  std::string pathFrame_, activePathFileName_, altPathFileName_, leftBoundFileName_,
      leftAltBoundFileName_, rightBoundFileName_, rightAltBoundFileName_;

  // Used to swap altPath on the fly
  std::vector<std::vector<double>> standbyPath_, standbyLeftBound_, standbyRightBound_;

  // path switching conditions and indicator
  bool useAltPathBasestn_ = false, useAltPathRaceCtrl_ = false, onAltPath_ = false;
  // In meters and radians
  double pathSwitchLateralThresh_, pathSwitchHeadingThresh_;

  /**
   * @brief Which path we are on (init path / standby [Alternative] path).
   */
  ros::Publisher onAltPathPub_;

};  // class

}  // namespace planning

#endif  // _PATH_SERVER__PATH_SERVER_HPP_
