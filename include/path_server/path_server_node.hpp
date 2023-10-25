/**
 * @file path_server_node.hpp
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

// #define DEBUG

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "path_server/path_server.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using std_msgs::msg::Bool;

namespace planning {
class PathServerNode : public rclcpp::Node {
 public:
  explicit PathServerNode(const rclcpp::NodeOptions& options);

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

 private:
  std::vector<std::vector<double>> readPathFromCsv(const std::string& pathFileName);

  void onUpdate();

  void pubErrMsg(const std::string& description, const int8_t& level, const int8_t& lifetime);

  bool swapActivePathPrecond();

  void swapActivePath();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& parameters);

  /**
   * @brief path server instances
   */
  PathServer *pathServer_, *altPathServer_, *leftBoundaryServer_, *rightBoundaryServer_,
      *leftAltBoundaryServer_, *rightAltBoundaryServer_;

  std::unique_ptr<PeerHandle> pathHandle_, altPathHandle_;

  bool isClosedAltPath_, interpretYaw_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  rclcpp::TimerBase::SharedPtr pathPubTimer_{nullptr};

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
  rclcpp::Publisher<Bool>::SharedPtr onAltPathPub_;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;

};  // class

}  // namespace planning

#endif  // _PATH_SERVER__PATH_SERVER_HPP_
