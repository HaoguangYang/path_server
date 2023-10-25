/**
 * @file path_server_node.cpp
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

#include "path_server/path_server_node.hpp"

#include <chrono>
#include <ctime>

#include "GeographicLib/UTMUPS.hpp"

namespace planning {

PathServerNode::PathServerNode(const rclcpp::NodeOptions& options)
    : Node("path_server_node", options) {
  bool doInterp = this->declare_parameter<bool>("perform_interpolation", true);
  double interpStep = this->declare_parameter<double>("interpolation_step_length", 1.0);
  interpretYaw_ = this->declare_parameter<bool>("interpret_yaw", true);
  std::string onNewPath = this->declare_parameter<std::string>("on_receiving_new_path", "replace");

  // path attributes that are subject to change after pathServer_ instantiated
  pathFrame_ = this->declare_parameter<std::string>("path_file_reference_frame", "earth");
  // attributed used to initialize pathServer_ instances only, will not change.
  std::string utmFrame, localEnuFrame, mapFrame, odomFrame, targetFrame;
  targetFrame = this->declare_parameter<std::string>("target_reference_frame", "base_link");
  utmFrame = this->declare_parameter<std::string>("utm_frame_id", "utm");
  localEnuFrame = this->declare_parameter<std::string>("local_enu_frame_id", "local_enu");
  mapFrame = this->declare_parameter<std::string>("map_frame_id", "map");
  odomFrame = this->declare_parameter<std::string>("odom_frame_id", "odom");

  // base folder path is only used during initial loading
  std::string pathFolder_ = this->declare_parameter<std::string>("path_folder", "");
  if (pathFolder_.length() && pathFolder_.back() != '/') pathFolder_ += "/";

  // load path names and closure properties
  activePathFileName_ =
      this->declare_parameter<std::string>("center_path.active_profile.file_name");
  // Active path -- the properties are set up front, no need for a class variable.
  bool isClosedActivePath =
      this->declare_parameter<bool>("center_path.active_profile.is_closed_path", true);
  leftBoundFileName_ = this->declare_parameter<std::string>("left_bound.active_profile.file_name",
                                                            activePathFileName_);
  rightBoundFileName_ = this->declare_parameter<std::string>("right_bound.active_profile.file_name",
                                                             activePathFileName_);

  altPathFileName_ = this->declare_parameter<std::string>("center_path.standby_profile.file_name",
                                                          activePathFileName_);
  isClosedAltPath_ =
      this->declare_parameter<bool>("center_path.standby_profile.is_closed_path", true);
  leftAltBoundFileName_ = this->declare_parameter<std::string>(
      "left_bound.standby_profile.file_name", leftBoundFileName_);
  rightAltBoundFileName_ = this->declare_parameter<std::string>(
      "right_bound.standby_profile.file_name", rightBoundFileName_);

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Pitting parameters
  pathSwitchLateralThresh_ =
      this->declare_parameter<double>("path_switching_sensitivity.lateral_difference_meters", 0.5);
  pathSwitchHeadingThresh_ =
      this->declare_parameter<double>("path_switching_sensitivity.heading_difference_radians", 0.2);

  // prepare UTM scaling factor
  std::vector<double> datum_vals{};
  datum_vals = this->declare_parameter("datum", datum_vals);
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
      this->declare_parameter<std::vector<std::string>>("center_path.plugins",
                                                        std::vector<std::string>{});
  std::vector<std::string> leftBoundPluginNames = this->declare_parameter<std::vector<std::string>>(
      "left_bound.plugins", std::vector<std::string>{});
  std::vector<std::string> rightBoundPluginNames =
      this->declare_parameter<std::vector<std::string>>("right_bound.plugins",
                                                        std::vector<std::string>{});

  std::vector<std::pair<std::string, std::string>> centerPathPlugins, leftBoundPlugins,
      rightBoundPlugins;
  std::vector<bool> centerPluginActiveEna, centerPluginStdbyEna, leftPluginActiveEna,
      leftPluginStdbyEna, rightPluginActiveEna, rightPluginStdbyEna;

  centerPathPlugins.reserve(centerPathPluginNames.size());
  centerPluginActiveEna.reserve(centerPathPluginNames.size());
  centerPluginStdbyEna.reserve(centerPathPluginNames.size());
  for (const auto& name : centerPathPluginNames) {
    centerPathPlugins.emplace_back(name, this->declare_parameter<std::string>(name + ".plugin"));
    centerPluginActiveEna.emplace_back(
        this->declare_parameter<bool>(name + ".enabled_when_active", true));
    centerPluginStdbyEna.emplace_back(
        this->declare_parameter<bool>(name + ".enabled_when_standby", false));
  }
  leftBoundPlugins.reserve(leftBoundPluginNames.size());
  leftPluginActiveEna.reserve(leftBoundPluginNames.size());
  leftPluginStdbyEna.reserve(leftBoundPluginNames.size());
  for (const auto& name : leftBoundPluginNames) {
    leftBoundPlugins.emplace_back(name, this->declare_parameter<std::string>(name + ".plugin"));
    leftPluginActiveEna.emplace_back(
        this->declare_parameter<bool>(name + ".enabled_when_active", true));
    leftPluginStdbyEna.emplace_back(
        this->declare_parameter<bool>(name + ".enabled_when_standby", false));
  }
  rightBoundPlugins.reserve(rightBoundPluginNames.size());
  rightPluginActiveEna.reserve(rightBoundPluginNames.size());
  rightPluginStdbyEna.reserve(rightBoundPluginNames.size());
  for (const auto& name : rightBoundPluginNames) {
    rightBoundPlugins.emplace_back(name, this->declare_parameter<std::string>(name + ".plugin"));
    rightPluginActiveEna.emplace_back(
        this->declare_parameter<bool>(name + ".enabled_when_active", true));
    rightPluginStdbyEna.emplace_back(
        this->declare_parameter<bool>(name + ".enabled_when_standby", false));
  }

  // assign header frame ids for path representations
  pathServer_ = new PathServer(this, tfBuffer_.get(), "center", targetFrame, utmFrame, localEnuFrame,
                               mapFrame, odomFrame, onNewPath, doInterp, interpStep, interpretYaw_,
                               centerPathPlugins, utmScale);
  // initialize without properties
  while (rclcpp::ok() && !registerPath(pathServer_, pathFolder_ + activePathFileName_,
                                       isClosedActivePath, pathFrame_))
    ;

  altPathServer_ = new PathServer(this, tfBuffer_.get(), "center", targetFrame, utmFrame, localEnuFrame,
                                  mapFrame, odomFrame, onNewPath, doInterp, interpStep,
                                  interpretYaw_, centerPathPlugins, utmScale);
  while (rclcpp::ok() && !registerPath(altPathServer_, pathFolder_ + altPathFileName_,
                                       isClosedAltPath_, pathFrame_))
    ;

  leftBoundaryServer_ = new PathServer(this, tfBuffer_.get(), "left_boundary", targetFrame, utmFrame,
                                       localEnuFrame, mapFrame, odomFrame, onNewPath, doInterp,
                                       interpStep, interpretYaw_, leftBoundPlugins, utmScale);
  while (rclcpp::ok() && !registerPath(leftBoundaryServer_, pathFolder_ + leftBoundFileName_,
                                       isClosedActivePath, pathFrame_))
    ;

  rightBoundaryServer_ = new PathServer(this, tfBuffer_.get(), "right_boundary", targetFrame, utmFrame,
                                        localEnuFrame, mapFrame, odomFrame, onNewPath, doInterp,
                                        interpStep, interpretYaw_, rightBoundPlugins, utmScale);
  while (rclcpp::ok() && !registerPath(rightBoundaryServer_, pathFolder_ + rightBoundFileName_,
                                       isClosedActivePath, pathFrame_))
    ;

  leftAltBoundaryServer_ = new PathServer(this, tfBuffer_.get(), "left_boundary", targetFrame, utmFrame,
                                          localEnuFrame, mapFrame, odomFrame, onNewPath, doInterp,
                                          interpStep, interpretYaw_, leftBoundPlugins, utmScale);
  while (rclcpp::ok() && !registerPath(leftAltBoundaryServer_, pathFolder_ + leftAltBoundFileName_,
                                       isClosedAltPath_, pathFrame_))
    ;

  rightAltBoundaryServer_ = new PathServer(this, tfBuffer_.get(), "right_boundary", targetFrame, utmFrame,
                                           localEnuFrame, mapFrame, odomFrame, onNewPath, doInterp,
                                           interpStep, interpretYaw_, rightBoundPlugins, utmScale);
  while (rclcpp::ok() &&
         !registerPath(rightAltBoundaryServer_, pathFolder_ + rightAltBoundFileName_,
                       isClosedAltPath_, pathFrame_))
    ;

  // pair paths based on context
  pathHandle_ = std::make_unique<PeerHandle>(PathServer::setBonds(
      std::vector<PathServer*>{pathServer_, leftBoundaryServer_, rightBoundaryServer_},
      std::vector<uint8_t>{1, 0, 0}));
  if (pathHandle_->size() != 3) {
    RCLCPP_ERROR(this->get_logger(),
                 "Initial paths bonding FAILED. PeerHandle size is %ld, expected 3",
                 pathHandle_->size());
  } else {
    RCLCPP_INFO(this->get_logger(), "Initial paths %s, %s, %s are bonded.",
                pathHandle_->at(0)->getPathData().getName().c_str(),
                pathHandle_->at(1)->getPathData().getName().c_str(),
                pathHandle_->at(2)->getPathData().getName().c_str());
  }

  altPathHandle_ = std::make_unique<PeerHandle>(PathServer::setBonds(
      std::vector<PathServer*>{altPathServer_, leftAltBoundaryServer_, rightAltBoundaryServer_},
      std::vector<uint8_t>{1, 0, 0}));
  if (altPathHandle_->size() != 3) {
    RCLCPP_ERROR(this->get_logger(),
                 "Standby paths bonding FAILED. PeerHandle size is %ld, expected 3",
                 altPathHandle_->size());
  } else {
    RCLCPP_INFO(this->get_logger(), "Standby paths %s, %s, %s are bonded.",
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
  while (rclcpp::ok()) {
    try {
      bool updatedPath = pathServer_->updateIndex();
      if (!updatedPath) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
      RCLCPP_WARN(this->get_logger(), "PathServerNode: waiting for TF buffer to populate: %s",
                  e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  // Setup callbacks
  pathPubTimer_ = this->create_wall_timer(0.01s, std::bind(&PathServerNode::onUpdate, this));

  paramCbHandle_ = this->add_on_set_parameters_callback(
      std::bind(&PathServerNode::parametersCallback, this, _1));
}

std::vector<std::vector<double>> PathServerNode::readPathFromCsv(const std::string& pathFileName) {
  std::vector<std::vector<double>> res;
  std::ifstream pathFileCsv;
  pathFileCsv.open(pathFileName.c_str());
  if (!pathFileCsv.is_open()) {
    RCLCPP_FATAL(this->get_logger(),
                 "PROVIDED PATH FILE NAME IS INVALID. THE NODE WILL NOT OUTPUT ANYTHING.");
    pubErrMsg("Provided path file name is invalid. The node will not output anything.", 5, 100);
    return res;
  }
  // parse csv file to x_file_, y_file_, yaw_file_ (unprocessed coords)
  std::string line;
  int lineN = 0;
  while (getline(pathFileCsv, line)) {
    lineN++;
    if (line.empty())  // skip empty lines:
      continue;
    // std::cout << line << std::endl;
    std::istringstream lineStream(line);
    std::string field;
    std::vector<double> row;
    try {
      while (getline(lineStream, field, ',')) {
        row.emplace_back(std::stod(field));  // convert to double
      }
      // std::cout << row[0] << ' ' << row[1] << std::endl;
    } catch (const std::invalid_argument& ia) {
      // this line has invalid number. Skip.
      RCLCPP_WARN(this->get_logger(),
                  "File %s Line %i is invalid (non-csv or non-numeric) and will be skipped.",
                  pathFileName.c_str(), lineN);
      continue;
    }
    res.emplace_back(row);
  }
  // we are done with the file.
  pathFileCsv.close();
  return res;
}

bool PathServerNode::registerPath(PathServer* ps, const std::string& pathFileName,
                                  const bool& isPathClosed, const std::string& pathFrame) {
  std::vector<std::vector<double>> rawCoords;
  RCLCPP_INFO(this->get_logger(), "Initializing path %s .", pathFileName.c_str());
  std::clock_t tic = std::clock();
  rawCoords = readPathFromCsv(pathFileName);
  // handle error
  if (!rawCoords.size()) {
    RCLCPP_FATAL(this->get_logger(),
                 "Reading path file %s FAILED! Please check file existance and formatting.",
                 pathFileName.c_str());
    pubErrMsg("Reading path file FAILED! Please check file existance and formatting.", 5, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return false;
  }
  try {
    ps->registerPath(pathFrame, rawCoords, isPathClosed, pathFileName);
  } catch (std::runtime_error& e) {
    RCLCPP_FATAL(
        this->get_logger(),
        "Registration of path FAILED! The specified path frame is invalid or not trasformable.");
    pubErrMsg(
        "Registration of path FAILED! The specified path frame is invalid or not trasformable.", 5,
        15);
    return false;
  }
  double toc = (std::clock() - tic) / (double)CLOCKS_PER_SEC;
  RCLCPP_INFO(this->get_logger(), "Path %s registered with %i points in %f seconds.",
              pathFileName.c_str(), static_cast<int>(rawCoords.size()), toc);
  return true;
}

void PathServerNode::onUpdate() {
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

  auto onAltPath = std::make_unique<Bool>();
  onAltPath->data = onAltPath_;
  onAltPathPub_->publish(std::move(onAltPath));

  if (swapActivePathPrecond()) swapActivePath();
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

rcl_interfaces::msg::SetParametersResult PathServerNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  // TODO: not all volatile parameters are mapped.
  for (const auto& param : parameters) {
    const std::string param_name = param.get_name();
    if (param_name == "path_file_reference_frame") {
      pathFrame_ = param.as_string();
      result.successful = true;
      result.reason += "Set path file reference frame to " + pathFrame_ + " . ";
    } else if (param_name == "center_path.standby_profile.is_closed_path") {
      isClosedAltPath_ = param.as_bool();
      result.successful = true;
      result.reason +=
          isClosedAltPath_ ? "Set path closure to TRUE." : "Set path closure to FALSE.";
    } else if (param_name == "center_path.standby_profile.file_name") {
      std::string newPath = param.as_string();
      if (newPath != altPathFileName_) {
        standbyPath_ = readPathFromCsv(newPath);
        if (!standbyPath_.size()) {
          result.successful = false;
          result.reason += "Failed to load path file " + newPath + " . ";
          return result;
        }
      }
      altPathFileName_ = newPath;
      try {
        altPathServer_->registerPath(pathFrame_, standbyPath_, isClosedAltPath_);
      } catch (std::runtime_error& e) {
        result.successful = false;
        result.reason +=
            "Failed to register path " + newPath + " to path server due to invalid frames.";
        return result;
      }
      if (altPathServer_->getCurrentInd() < 0) {
        result.successful = false;
        result.reason += "Failed to register path " + newPath + " to path server. ";
        return result;
      }
      result.successful = true;
      result.reason += "Loaded and registered standby path " + newPath + " . ";
    } else if (param_name == "left_bound.standby_profile.file_name") {
      std::string newLeftBound = param.as_string();
      if (newLeftBound != leftAltBoundFileName_) {
        standbyLeftBound_ = readPathFromCsv(newLeftBound);
        if (!standbyLeftBound_.size()) {
          result.successful = false;
          result.reason += "Failed to load path file " + newLeftBound + " . ";
          return result;
        }
      }
      leftAltBoundFileName_ = newLeftBound;
      try {
        leftAltBoundaryServer_->registerPath(pathFrame_, standbyLeftBound_, isClosedAltPath_);
      } catch (std::runtime_error& e) {
        result.successful = false;
        result.reason +=
            "Failed to register path " + newLeftBound + " to path server due to invalid frames.";
        return result;
      }
      if (leftAltBoundaryServer_->getCurrentInd() < 0) {
        result.successful = false;
        result.reason += "Failed to register path " + newLeftBound + " to path server. ";
        return result;
      }
      result.successful = true;
      result.reason += "Set standby left boundary file name to " + newLeftBound + " . ";
    } else if (param_name == "right_bound.standby_profile.file_name") {
      std::string newRightBound = param.as_string();
      if (newRightBound != rightAltBoundFileName_) {
        standbyRightBound_ = readPathFromCsv(newRightBound);
        if (!standbyRightBound_.size()) {
          result.successful = false;
          result.reason += "Failed to load path file " + newRightBound + " . ";
          return result;
        }
      }
      rightAltBoundFileName_ = newRightBound;
      try {
        rightAltBoundaryServer_->registerPath(pathFrame_, standbyRightBound_, isClosedAltPath_);
      } catch (std::runtime_error& e) {
        result.successful = false;
        result.reason +=
            "Failed to register path " + newRightBound + " to path server due to invalid frames.";
        return result;
      }
      if (rightAltBoundaryServer_->getCurrentInd() < 0) {
        result.successful = false;
        result.reason += "Failed to register path " + newRightBound + " to path server. ";
        return result;
      }
      result.successful = true;
      result.reason += "Set standby right boundary file name to " + newRightBound + " . ";
    }
  }
  if (result.reason == "" && !(result.successful))
    result.reason = "Trying to update an unsupported parameter.";
  return result;
}

void PathServerNode::pubErrMsg(const std::string& description, const int8_t& level,
                               const int8_t& lifetime) {
  // FIXME: Something went wrong. Need to report to the remote station.
  (void) description;
  (void) level;
  (void) lifetime;
}

}  // namespace planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planning::PathServerNode)
