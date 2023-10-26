/**
 * @file path_server_node_ros1.cpp
 * @author Haoguang Yang (yang1510@purdue.edu)
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

#include "path_server/path_server_node_ros1.hpp"

#include <chrono>
#include <ctime>

namespace planning {

PathServerNode::PathServerNode(ros::NodeHandle* nh) : node_(nh) {
  bool doInterp = node_->param<bool>("perform_interpolation", true);
  double interpStep = node_->param<double>("interpolation_step_length", 1.0);
  interpretYaw_ = node_->param<bool>("interpret_yaw", true);
  std::string onNewPath = node_->param<std::string>("on_receiving_new_path", "replace");

  // path attributes that are subject to change after pathServer_ instantiated
  pathFrame_ = node_->param<std::string>("path_file_reference_frame", "earth");
  // attributed used to initialize pathServer_ instances only, will not change.
  std::string utmFrame, localEnuFrame, mapFrame, odomFrame, targetFrame;
  targetFrame = node_->param<std::string>("target_reference_frame", "base_link");
  utmFrame = node_->param<std::string>("utm_frame_id", "utm");
  localEnuFrame = node_->param<std::string>("local_enu_frame_id", "local_enu");
  mapFrame = node_->param<std::string>("map_frame_id", "map");
  odomFrame = node_->param<std::string>("odom_frame_id", "odom");

  // base folder path is only used during initial loading
  std::string pathFolder_ = node_->param<std::string>("path_folder", "");
  if (pathFolder_.length() && pathFolder_.back() != '/') pathFolder_ += "/";

  // load path names and closure properties
  node_->getParam("center_path/active_profile/file_name", activePathFileName_);
  // Active path -- the properties are set up front, no need for a class variable.
  bool isClosedActivePath = node_->param<bool>("center_path/active_profile/is_closed_path", true);
  leftBoundFileName_ =
      node_->param<std::string>("left_bound/active_profile/file_name", activePathFileName_);
  rightBoundFileName_ =
      node_->param<std::string>("right_bound/active_profile/file_name", activePathFileName_);

  altPathFileName_ =
      node_->param<std::string>("center_path/standby_profile/file_name", activePathFileName_);
  isClosedAltPath_ = node_->param<bool>("center_path/standby_profile/is_closed_path", true);
  leftAltBoundFileName_ =
      node_->param<std::string>("left_bound/standby_profile/file_name", leftBoundFileName_);
  rightAltBoundFileName_ =
      node_->param<std::string>("right_bound/standby_profile/file_name", rightBoundFileName_);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>();
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Pitting parameters
  pathSwitchLateralThresh_ =
      node_->param<double>("path_switching_sensitivity/lateral_difference_meters", 0.5);
  pathSwitchHeadingThresh_ =
      node_->param<double>("path_switching_sensitivity/heading_difference_radians", 0.2);

  // prepare UTM scaling factor
  std::vector<double> datum_vals{};
  datum_vals = node_->param<std::vector<double>>("datum", datum_vals);
  double utmScale = 1.0;
  if (datum_vals.size() >= 2) {
    double datum_lat = datum_vals[0];
    double datum_lon = datum_vals[1];
    int utm_zone_tmp;
    bool northp_tmp;
    double utm_e_tmp, utm_n_tmp, gamma_tmp;
    try {
      GeographicLib::UTMUPS::Forward(datum_lat, datum_lon, utm_zone_tmp, northp_tmp, utm_e_tmp,
                                     utm_n_tmp, gamma_tmp, utmScale);
    } catch (GeographicLib::GeographicErr& ex) {
      // lat/lon combination is invalid
    }
  }

  // load plugin name/library pairs
  std::vector<std::string> centerPathPluginNames =
      node_->param<std::vector<std::string>>("center_path.plugins", std::vector<std::string>{});
  std::vector<std::string> leftBoundPluginNames =
      node_->param<std::vector<std::string>>("left_bound.plugins", std::vector<std::string>{});
  std::vector<std::string> rightBoundPluginNames =
      node_->param<std::vector<std::string>>("right_bound.plugins", std::vector<std::string>{});

  std::vector<std::pair<std::string, std::string>> centerPathPlugins, leftBoundPlugins,
      rightBoundPlugins;
  std::vector<bool> centerPluginActiveEna, centerPluginStdbyEna, leftPluginActiveEna,
      leftPluginStdbyEna, rightPluginActiveEna, rightPluginStdbyEna;

  centerPathPlugins.reserve(centerPathPluginNames.size());
  centerPluginActiveEna.reserve(centerPathPluginNames.size());
  centerPluginStdbyEna.reserve(centerPathPluginNames.size());
  for (const auto& name : centerPathPluginNames) {
    std::string pluginName;
    node_->getParam(name + "/plugin", pluginName);
    centerPathPlugins.emplace_back(name, pluginName);
    centerPluginActiveEna.emplace_back(node_->param<bool>(name + "/enabled_when_active", true));
    centerPluginStdbyEna.emplace_back(node_->param<bool>(name + "/enabled_when_standby", false));
  }
  leftBoundPlugins.reserve(leftBoundPluginNames.size());
  leftPluginActiveEna.reserve(leftBoundPluginNames.size());
  leftPluginStdbyEna.reserve(leftBoundPluginNames.size());
  for (const auto& name : leftBoundPluginNames) {
    std::string pluginName;
    node_->getParam(name + "/plugin", pluginName);
    leftBoundPlugins.emplace_back(name, pluginName);
    leftPluginActiveEna.emplace_back(node_->param<bool>(name + "/enabled_when_active", true));
    leftPluginStdbyEna.emplace_back(node_->param<bool>(name + "/enabled_when_standby", false));
  }
  rightBoundPlugins.reserve(rightBoundPluginNames.size());
  rightPluginActiveEna.reserve(rightBoundPluginNames.size());
  rightPluginStdbyEna.reserve(rightBoundPluginNames.size());
  for (const auto& name : rightBoundPluginNames) {
    std::string pluginName;
    node_->getParam(name + "/plugin", pluginName);
    rightBoundPlugins.emplace_back(name, pluginName);
    rightPluginActiveEna.emplace_back(node_->param<bool>(name + "/enabled_when_active", true));
    rightPluginStdbyEna.emplace_back(node_->param<bool>(name + "/enabled_when_standby", false));
  }

  // assign header frame ids for path representations
  pathServer_ = new PathServer(node_, tfBuffer_.get(), "center", targetFrame, utmFrame,
                               localEnuFrame, mapFrame, odomFrame, onNewPath, doInterp, interpStep,
                               interpretYaw_, centerPathPlugins, utmScale);
  // initialize without properties
  while (node_->ok() && !registerPath(pathServer_, pathFolder_ + activePathFileName_,
                                      isClosedActivePath, pathFrame_))
    ;

  altPathServer_ = new PathServer(node_, tfBuffer_.get(), "center", targetFrame, utmFrame,
                                  localEnuFrame, mapFrame, odomFrame, onNewPath, doInterp,
                                  interpStep, interpretYaw_, centerPathPlugins, utmScale);
  while (node_->ok() && !registerPath(altPathServer_, pathFolder_ + altPathFileName_,
                                      isClosedAltPath_, pathFrame_))
    ;

  leftBoundaryServer_ = new PathServer(
      node_, tfBuffer_.get(), "left_boundary", targetFrame, utmFrame, localEnuFrame, mapFrame,
      odomFrame, onNewPath, doInterp, interpStep, interpretYaw_, leftBoundPlugins, utmScale);
  while (node_->ok() && !registerPath(leftBoundaryServer_, pathFolder_ + leftBoundFileName_,
                                      isClosedActivePath, pathFrame_))
    ;

  rightBoundaryServer_ = new PathServer(
      node_, tfBuffer_.get(), "right_boundary", targetFrame, utmFrame, localEnuFrame, mapFrame,
      odomFrame, onNewPath, doInterp, interpStep, interpretYaw_, rightBoundPlugins, utmScale);
  while (node_->ok() && !registerPath(rightBoundaryServer_, pathFolder_ + rightBoundFileName_,
                                      isClosedActivePath, pathFrame_))
    ;

  leftAltBoundaryServer_ = new PathServer(
      node_, tfBuffer_.get(), "left_boundary", targetFrame, utmFrame, localEnuFrame, mapFrame,
      odomFrame, onNewPath, doInterp, interpStep, interpretYaw_, leftBoundPlugins, utmScale);
  while (node_->ok() && !registerPath(leftAltBoundaryServer_, pathFolder_ + leftAltBoundFileName_,
                                      isClosedAltPath_, pathFrame_))
    ;

  rightAltBoundaryServer_ = new PathServer(
      node_, tfBuffer_.get(), "right_boundary", targetFrame, utmFrame, localEnuFrame, mapFrame,
      odomFrame, onNewPath, doInterp, interpStep, interpretYaw_, rightBoundPlugins, utmScale);
  while (node_->ok() && !registerPath(rightAltBoundaryServer_, pathFolder_ + rightAltBoundFileName_,
                                      isClosedAltPath_, pathFrame_))
    ;

  // pair paths based on context
  pathHandle_ = std::make_unique<PeerHandle>(PathServer::setBonds(
      std::vector<PathServer*>{pathServer_, leftBoundaryServer_, rightBoundaryServer_},
      std::vector<uint8_t>{1, 0, 0}));
  if (pathHandle_->size() != 3) {
    ROS_ERROR("Initial paths bonding FAILED. PeerHandle size is %ld, expected 3",
              pathHandle_->size());
  } else {
    ROS_INFO("Initial paths %s, %s, %s are bonded.",
             pathHandle_->at(0)->getPathData().getName().c_str(),
             pathHandle_->at(1)->getPathData().getName().c_str(),
             pathHandle_->at(2)->getPathData().getName().c_str());
  }

  altPathHandle_ = std::make_unique<PeerHandle>(PathServer::setBonds(
      std::vector<PathServer*>{altPathServer_, leftAltBoundaryServer_, rightAltBoundaryServer_},
      std::vector<uint8_t>{1, 0, 0}));
  if (altPathHandle_->size() != 3) {
    ROS_ERROR("Standby paths bonding FAILED. PeerHandle size is %ld, expected 3",
              altPathHandle_->size());
  } else {
    ROS_INFO("Standby paths %s, %s, %s are bonded.",
             altPathHandle_->at(0)->getPathData().getName().c_str(),
             altPathHandle_->at(1)->getPathData().getName().c_str(),
             altPathHandle_->at(2)->getPathData().getName().c_str());
  }

  // initialize properties
  pathHandle_->configureProperties();
  altPathHandle_->configureProperties();

  // enable plugins as specified
  pathServer_->setPluginActivationStates(centerPluginActiveEna);
  altPathServer_->setPluginActivationStates(centerPluginStdbyEna);
  leftBoundaryServer_->setPluginActivationStates(leftPluginActiveEna);
  rightBoundaryServer_->setPluginActivationStates(rightPluginActiveEna);
  leftAltBoundaryServer_->setPluginActivationStates(leftPluginStdbyEna);
  rightAltBoundaryServer_->setPluginActivationStates(rightPluginStdbyEna);

  this->onAltPathPub_ = node_->advertise<Bool>("on_alt_path", 5);

  // Now we can update both path server and alternative path server.
  // Test current distance to both paths. If we are closer to alternative path instead,
  // we assume we started on the alternative path.
  const PoseStamped self = [&]() {
    PoseStamped ret;
    // ret.header.stamp = now();  // you shouldn't do this.
    ret.header.frame_id = targetFrame;
    ret.pose.position.x = 0.0;
    ret.pose.position.y = 0.0;
    ret.pose.position.z = 0.0;
    ret.pose.orientation.x = 0.0;
    ret.pose.orientation.y = 0.0;
    ret.pose.orientation.z = 0.0;
    ret.pose.orientation.w = 1.0;
    return ret;
  }();
  while (node_->ok()) {
    try {
      bool updatedPath = pathServer_->updateIndex();
      if (!updatedPath) {
        ros::Duration(0.2).sleep();
        continue;
      }
      updatedPath = altPathServer_->updateIndex();
      // If alternative path is not ready, stick to initial path.
      if (!updatedPath) break;
      auto poseToInitPath =
          path_utils::safeProjection(*tfBuffer_, self, pathServer_->getClosestWaypoint());
      auto poseToAltPath =
          path_utils::safeProjection(*tfBuffer_, self, altPathServer_->getClosestWaypoint());
      if (std::fabs(poseToAltPath.position.y) < std::fabs(poseToInitPath.position.y))
        swapActivePath();
      break;
    } catch (tf2::TransformException& e) {
      ROS_WARN("PathServerNode: waiting for TF buffer to populate: %s", e.what());
      ros::Duration(0.2).sleep();
    }
  }

  // Setup update loop
  updateTimer_ = std::make_unique<ros::Rate>(100.0);
}

bool PathServerNode::registerPath(PathServer* ps, const std::string& pathFileName,
                                  const bool& isPathClosed, const std::string& pathFrame) {
  std::vector<std::vector<double>> rawCoords;
  ROS_INFO("Initializing path %s .", pathFileName.c_str());
  std::clock_t tic = std::clock();
  rawCoords = path_utils::readMatrixFromCsv(node_, pathFileName);
  // handle error
  if (!rawCoords.size()) {
    std::string msg =
        "Reading path file" + pathFileName + "FAILED! Please check file existance and formatting.";
    ROS_FATAL(msg.c_str());
    pubErrMsg(msg, 5, 100);
    ros::Duration(1.0).sleep();
    return false;
  }
  try {
    ps->registerPath(pathFrame, rawCoords, isPathClosed, pathFileName);
  } catch (std::runtime_error& e) {
    std::string msg =
        "Registration of path FAILED! The specified path frame is invalid or not trasformable.";
    ROS_FATAL(msg.c_str());
    pubErrMsg(msg, 5, 15);
    return false;
  }
  double toc = (std::clock() - tic) / (double)CLOCKS_PER_SEC;
  ROS_INFO("Path %s registered with %i points in %f seconds.", pathFileName.c_str(),
           static_cast<int>(rawCoords.size()), toc);
  return true;
}

/**
 * publish path in target_frame, extracting the segment within
 * [lookBehind, lookAhead]
 */
void PathServerNode::run() {
  while (node_->ok()) {
    // update active path
    bool pathUpdated = pathHandle_->update();
    if (!pathUpdated) {
      pubErrMsg("Path server failed to update! Please check path and frame validity!", 4, 1);
    }
    // update standby path
    bool altPathUpdated = altPathHandle_->update();
    if (!altPathUpdated) {
      pubErrMsg(
          "Path server for alternative path failed to update! Please check alt path and frame "
          "validity!",
          2, 1);
    }

    auto onAltPath = Bool();
    onAltPath.data = onAltPath_;
    onAltPathPub_.publish(onAltPath);

    if (swapActivePathPrecond()) swapActivePath();
    updateTimer_->sleep();
  }
}

bool PathServerNode::swapActivePathPrecond() {
  // FIXME: provide your condition for swapping the active path and the standby path.
  return false;
}

void PathServerNode::swapActivePath() {
  std::vector<bool> psAct = pathServer_->getPluginActivationStates();
  std::vector<bool> psAlt = altPathServer_->getPluginActivationStates();
  std::swap(pathServer_, altPathServer_);
  pathServer_->setPluginActivationStates(psAct);
  altPathServer_->setPluginActivationStates(psAlt);

  std::vector<bool> lAct = leftBoundaryServer_->getPluginActivationStates();
  std::vector<bool> lAlt = leftAltBoundaryServer_->getPluginActivationStates();
  std::swap(leftBoundaryServer_, leftAltBoundaryServer_);
  leftBoundaryServer_->setPluginActivationStates(lAct);
  leftAltBoundaryServer_->setPluginActivationStates(lAlt);

  std::vector<bool> rAct = rightBoundaryServer_->getPluginActivationStates();
  std::vector<bool> rAlt = rightAltBoundaryServer_->getPluginActivationStates();
  std::swap(rightBoundaryServer_, rightAltBoundaryServer_);
  rightBoundaryServer_->setPluginActivationStates(rAct);
  rightAltBoundaryServer_->setPluginActivationStates(rAlt);

  std::swap(pathHandle_, altPathHandle_);
  onAltPath_ = !onAltPath_;
}

void PathServerNode::pubErrMsg(const std::string& description, const int8_t& level,
                               const int8_t& lifetime) {
  // FIXME: Something went wrong. Need to report to the remote station.
  (void) description;
  (void) level;
  (void) lifetime;
}

}  // namespace planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_server");
  ros::NodeHandle n;
  planning::PathServerNode pathServerInst(&n);
  pathServerInst.run();
  return 0;
}
