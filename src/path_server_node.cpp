#include "path_server/path_server_node.hpp"

#include <chrono>
#include <ctime>

namespace path_server {

PathServerNode::PathServerNode(const rclcpp::NodeOptions& options)
    : Node("path_server_node", options) {
  // path attributes that are subject to change after pathServer_ instantiated
  pathFrame_ = this->declare_parameter<std::string>("path_file_reference_frame", "earth");

  // base folder path is only used during initial loading
  std::string pathFolder_ = this->declare_parameter<std::string>("path_folder", "");
  if (pathFolder_.length() && pathFolder_.back() != '/') pathFolder_ += "/";

  activePathFileName_ = this->declare_parameter<std::string>("init_path_file_name");
  altPathFileName_ =
      this->declare_parameter<std::string>("alt_path_file_name", activePathFileName_);

  leftBoundFileName_ = this->declare_parameter<std::string>("init_left_bound_file_name");
  leftAltBoundFileName_ =
      this->declare_parameter<std::string>("alt_left_bound_file_name", leftBoundFileName_);

  rightBoundFileName_ = this->declare_parameter<std::string>("init_right_bound_file_name");
  rightAltBoundFileName_ =
      this->declare_parameter<std::string>("alt_right_bound_file_name", rightBoundFileName_);

  lookAheadDist_ = this->declare_parameter<double>("look_ahead_distance", 100.0);
  lookBehindDist_ = this->declare_parameter<double>("look_behind_distance", 0.0);
  pathOffsetRateLimit_ = this->declare_parameter<double>("lateral_offset_rate_limit", 0.1);
  isClosedPath_ = this->declare_parameter<bool>("is_closed_path", true);
  doInterp_ = this->declare_parameter<bool>("perform_interpolation", true);
  interpStep_ = this->declare_parameter<double>("interpolation_step_length", 1.0);
  interpretYaw_ = this->declare_parameter<bool>("interpret_yaw", true);

  // attributed used to initialize pathServer_ instances only, will not change.
  std::string utmFrame, localEnuFrame, mapFrame, odomFrame, targetFrame, onNewPath;
  targetFrame = this->declare_parameter<std::string>("target_reference_frame", "base_link");
  utmFrame = this->declare_parameter<std::string>("utm_frame_id", "utm");
  localEnuFrame = this->declare_parameter<std::string>("local_enu_frame_id", "local_enu");
  mapFrame = this->declare_parameter<std::string>("map_frame_id", "map");
  odomFrame = this->declare_parameter<std::string>("odom_frame_id", "odom");
  onNewPath = this->declare_parameter<std::string>("on_receiving_new_path", "replace");

  frontPathTopic_ = this->declare_parameter<std::string>("front_path_topic", "front_path");
  rearPathTopic_ = this->declare_parameter<std::string>("rear_path_topic", "rear_path");
  this->frontPathPub_ =
      this->create_publisher<Path>(frontPathTopic_ + "/path", rclcpp::SystemDefaultsQoS());
  this->rearPathPub_ =
      this->create_publisher<Path>(rearPathTopic_ + "/path", rclcpp::SystemDefaultsQoS());
  this->frontOffsetPathPub_ =
      this->create_publisher<Path>(frontPathTopic_ + "/offset_path", rclcpp::SystemDefaultsQoS());
  this->rearOffsetPathPub_ =
      this->create_publisher<Path>(rearPathTopic_ + "/offset_path", rclcpp::SystemDefaultsQoS());

  this->leftBoundaryPub_ =
      this->create_publisher<Path>("left_boundary", rclcpp::SystemDefaultsQoS());
  this->rightBoundaryPub_ =
      this->create_publisher<Path>("right_boundary", rclcpp::SystemDefaultsQoS());

  this->inPitLanePub_ = this->create_publisher<Bool>("in_pit_lane", rclcpp::SystemDefaultsQoS());
  this->offsetRangePub_ = this->create_publisher<Float32MultiArray>("feasible_lateral_offset_range",
                                                                    rclcpp::SystemDefaultsQoS());
  this->pathCurvaturePub_ =
      this->create_publisher<Float32MultiArray>("path_curvature", rclcpp::SystemDefaultsQoS());
  this->offsetDistPub_ =
      this->create_publisher<Float32MultiArray>("path_offset", rclcpp::SystemDefaultsQoS());
  // this->errPub_ = this->create_publisher<ErrorReport>("errors", rclcpp::SystemDefaultsQoS());

#ifdef DEBUG
  this->frontAltPathPub_ =
      this->create_publisher<Path>(frontPathTopic_ + "/alt_path", rclcpp::SystemDefaultsQoS());
#endif

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  geometry_msgs::msg::TransformStamped t;

  // assign header frame ids for path representations
  // cast unique_ptr to raw pointer as tfBuffer_ will continue to exist until after pathServer_ gets
  // destroyed.
  pathServer_ = new PathServer(this, tfBuffer_.get(), targetFrame, utmFrame, localEnuFrame,
                               mapFrame, odomFrame, onNewPath);
  pathServer_->setPathAttr(lookAheadDist_, lookBehindDist_, isClosedPath_, doInterp_, interpStep_,
                           interpretYaw_, true);
  pathServer_->setOffsetRateLimit(static_cast<float>(pathOffsetRateLimit_));
  while (!initialize(pathServer_, pathFolder_ + activePathFileName_, pathFrame_))
    ;

  altPathServer_ = new PathServer(this, tfBuffer_.get(), targetFrame, utmFrame, localEnuFrame,
                                  mapFrame, odomFrame, onNewPath);
#ifdef DEBUG
  altPathServer_->setPathAttr(lookAheadDist_, lookBehindDist_, isClosedPath_, doInterp_,
                              interpStep_, interpretYaw_, true);
#else
  altPathServer_->setPathAttr(lookAheadDist_, lookBehindDist_, isClosedPath_, doInterp_,
                              interpStep_, interpretYaw_, false);
#endif
  altPathServer_->setOffsetRateLimit(static_cast<float>(pathOffsetRateLimit_));
  while (!initialize(altPathServer_, pathFolder_ + altPathFileName_, pathFrame_))
    ;

  leftBoundaryServer_ = new PathServer(this, tfBuffer_.get(), targetFrame, utmFrame, localEnuFrame,
                                       mapFrame, odomFrame, onNewPath);
  leftBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                   interpretYaw_, true);
  while (!initialize(leftBoundaryServer_, pathFolder_ + leftBoundFileName_, pathFrame_))
    ;

  rightBoundaryServer_ = new PathServer(this, tfBuffer_.get(), targetFrame, utmFrame, localEnuFrame,
                                        mapFrame, odomFrame, onNewPath);
  rightBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                    interpretYaw_, true);
  while (!initialize(rightBoundaryServer_, pathFolder_ + rightBoundFileName_, pathFrame_))
    ;

  leftAltBoundaryServer_ = new PathServer(this, tfBuffer_.get(), targetFrame, utmFrame,
                                          localEnuFrame, mapFrame, odomFrame, onNewPath);
  leftAltBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                      interpretYaw_, false);
  while (!initialize(leftAltBoundaryServer_, pathFolder_ + leftAltBoundFileName_, pathFrame_))
    ;

  rightAltBoundaryServer_ = new PathServer(this, tfBuffer_.get(), targetFrame, utmFrame,
                                           localEnuFrame, mapFrame, odomFrame, onNewPath);
  rightAltBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                       interpretYaw_, false);
  while (!initialize(rightAltBoundaryServer_, pathFolder_ + rightAltBoundFileName_, pathFrame_))
    ;

  pathPubTimer_ = this->create_wall_timer(0.01s, std::bind(&PathServerNode::onPathPublish, this));

  /*
  pathOffsetSub_ = this->create_subscription<PathOffsetCommand>(
      "path_offset_command", rclcpp::SystemDefaultsQoS(),
      std::bind(&PathServerNode::onPathOffsetCmd, this, _1));

  // basestation commands
  this->pathSwitchingSub_ = this->create_subscription<BasestationCommand>(
      "/basestation/command", rclcpp::SystemDefaultsQoS(),
      std::bind(&PathServerNode::onPathSwitchingCmd, this, _1));

  // flag signals
  // basestation commands
  this->flagSigSub_ = this->create_subscription<RcToCt>(
      "/raptor_dbw_interface/rc_to_ct", rclcpp::SystemDefaultsQoS(),
      std::bind(&PathServerNode::onFlagSignal, this, _1));
  */

  paramCbHandle_ = this->add_on_set_parameters_callback(
      std::bind(&PathServerNode::parametersCallback, this, _1));
}

std::vector<std::vector<double>> PathServerNode::readPathFromCsv(const std::string& pathFileName) {
  std::vector<std::vector<double>> res;
  std::ifstream pathFileCsv_;
  pathFileCsv_.open(pathFileName.c_str());
  if (!pathFileCsv_.is_open()) {
    RCLCPP_FATAL(this->get_logger(),
                 "PROVIDED PATH FILE NAME IS INVALID. THE NODE WILL NOT OUTPUT ANYTHING.");
    pubErrMsg("Provided path file name is invalid. The node will not output anything.", 5, 100);
    return res;
  }
  // parse csv file to x_file_, y_file_, yaw_file_ (unprocessed coords)
  std::string line;
  int lineN = 0;
  while (getline(pathFileCsv_, line)) {
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
  pathFileCsv_.close();
  return res;
}

bool PathServerNode::initialize(PathServer* ps, const std::string& pathFileName,
                                const std::string& pathFrame) {
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
    ps->registerPath(pathFrame, rawCoords);
  } catch (std::runtime_error& e) {
    RCLCPP_FATAL(
        this->get_logger(),
        "Registration of path FAILED! The specified path frame is invalid or not trasformable.");
    pubErrMsg(
        "Registration of path FAILED! The specified path frame is invalid or not trasformable.", 5,
        15);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Path %s registered with %i points.", pathFileName.c_str(),
              static_cast<int>(rawCoords.size()));

  // initialization process guarantees a non-negative currentInd_
  while (rclcpp::ok() && ps->getCurrentInd() < 0) {
    // initialization of frenet frame and register path
    ps->initialize();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  double toc = (std::clock() - tic) / (double)CLOCKS_PER_SEC;
  RCLCPP_INFO(this->get_logger(), "Initialized path %s with length %f meters in %f seconds.",
              pathFileName.c_str(), ps->getPathCumulativeDist(), toc);
  return true;
}

/**
 * publish path in target_frame, extracting the segment within
 * [lookBehind, lookAhead]
 */
void PathServerNode::onPathPublish() {
  bool updatedPath = pathServer_->updateStep();
  if (!updatedPath) {
    pubErrMsg("Path server failed to update! Please check path and frame validity!", 4, 1);
  }
  altPathServer_->updateStep();
  bool updatedLeftBound = leftBoundaryServer_->updateStep();
  bool updatedRightBound = rightBoundaryServer_->updateStep();
  auto frontPath = std::make_unique<Path>(pathServer_->getFrontPath());
  // error handling for empty path
  if (!frontPath->poses.size()) {
    pubErrMsg("Path has run out! <error code 132352>", 5, 1);
  }
  frontPathPub_->publish(std::move(frontPath));
  rearPathPub_->publish(pathServer_->getRearPath());
  frontOffsetPathPub_->publish(pathServer_->getFrontOffsetPath());
  rearOffsetPathPub_->publish(pathServer_->getRearOffsetPath());
#ifdef DEBUG
  frontAltPathPub_->publish(altPathServer_->getFrontOffsetPath());
#endif
  leftBoundaryPub_->publish(leftBoundaryServer_->getFrontOffsetPath());
  rightBoundaryPub_->publish(rightBoundaryServer_->getFrontOffsetPath());

  // publish path curvature
  auto curvatureMsg = std::make_unique<Float32MultiArray>();
  curvatureMsg->data = pathServer_->getFrontCurvature();
  pathCurvaturePub_->publish(std::move(curvatureMsg));

  // publish offset array
  auto offsetArray = std::make_unique<Float32MultiArray>();
  offsetArray->data = pathServer_->getFrontOffsetArray();
  offsetDistPub_->publish(std::move(offsetArray));

  auto pitMsg = std::make_unique<Bool>();
  pitMsg->data = !onAltPath_;
  inPitLanePub_->publish(std::move(pitMsg));

  // This used to get called only when a path offset command has arrived.
  // Now it is moved to a periodic segment.
  // TODO: consider timing delays from header
  // check boundaries and clamp msg->leftwards_offset
  if (!updatedLeftBound) {
    maxLeftwardsOffset_ = 0.;
  } else {
    PoseStamped leftBound = leftBoundaryServer_->getClosestPose();
    Pose L = pathServer_->safeProjection(pathServer_->getClosestPose(), leftBound);
    maxLeftwardsOffset_ = static_cast<float>(L.position.y);
  }
  if (!updatedRightBound) {
    minLeftwardsOffset_ = 0.;
  } else {
    PoseStamped rightBound = rightBoundaryServer_->getClosestPose();
    Pose R = pathServer_->safeProjection(pathServer_->getClosestPose(), rightBound);
    minLeftwardsOffset_ = static_cast<float>(R.position.y);
  }
  // whatever offset should have zero enclosed. Check for inconsistencies. Otherwise disable offset.
  if (maxLeftwardsOffset_ < minLeftwardsOffset_ || maxLeftwardsOffset_ < 0. ||
      minLeftwardsOffset_ > 0.) {
    maxLeftwardsOffset_ = 0.;
    minLeftwardsOffset_ = 0.;
  }
  auto offsetRange = std::make_unique<Float32MultiArray>();
  // first element is max (left end), second element is min (right end).
  offsetRange->data.emplace_back(maxLeftwardsOffset_);
  offsetRange->data.emplace_back(minLeftwardsOffset_);
  offsetRangePub_->publish(std::move(offsetRange));

  if (swapActivePathPrecond()) swapActivePath();
}

bool PathServerNode::swapActivePathPrecond() {
  // if it is not requested, do not swap.
  if (useAltPath_ == onAltPath_) return false;
  // if alt is not properly initialized, do not swap.
  if (altPathServer_->getCurrentInd() < 0 || leftAltBoundaryServer_->getCurrentInd() < 0 ||
      rightAltBoundaryServer_->getCurrentInd() < 0)
    return false;
  // if lateral projection difference or heading difference is greater than threshold, do not swap
  PoseStamped pathPt = pathServer_->getClosestPose();
  PoseStamped altPathPt = altPathServer_->getClosestPose();
  Pose p = pathServer_->safeProjection(pathPt, altPathPt);
  double rot = 0.;
  if (interpretYaw_) {
    tf2::Quaternion q;
    tf2::fromMsg(p.orientation, q);
    rot = q.getAngleShortestPath();
  }
  // RCLCPP_INFO(this->get_logger(), "errors: %f, %f, %f", p.position.x, p.position.y, rot);
  //  TODO: parameterize this
  if (std::fabs(p.position.y) <= 0.5 && rot <= 0.2) return true;
  return false;
}

void PathServerNode::swapActivePath() {
  pathServer_->clearOffset();
  std::swap(pathServer_, altPathServer_);
  pathServer_->setPathAttr(lookAheadDist_, lookBehindDist_, isClosedPath_, doInterp_, interpStep_,
                           interpretYaw_, true);
#ifndef DEBUG
  altPathServer_->setPathAttr(lookAheadDist_, lookBehindDist_, isClosedPath_, doInterp_,
                              interpStep_, interpretYaw_, false);
#endif
  std::swap(leftBoundaryServer_, leftAltBoundaryServer_);
  leftBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                   interpretYaw_, true);
  leftAltBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                      interpretYaw_, false);
  std::swap(rightBoundaryServer_, rightAltBoundaryServer_);
  rightBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                    interpretYaw_, true);
  rightAltBoundaryServer_->setPathAttr(lookAheadDist_, 0., isClosedPath_, doInterp_, interpStep_,
                                       interpretYaw_, false);
  onAltPath_ = !onAltPath_;
}

rcl_interfaces::msg::SetParametersResult PathServerNode::parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  // TODO: not all volatile parameters are mapped.
  for (const auto& param : parameters) {
    if (param.get_name() == "alt_path_file_name") {
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
        altPathServer_->registerPath(pathFrame_, standbyPath_);
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
    } else if (param.get_name() == "path_file_reference_frame") {
      pathFrame_ = param.as_string();
      result.successful = true;
      result.reason += "Set path file reference frame to " + pathFrame_ + " . ";
    } else if (param.get_name() == "alt_left_bound_file_name") {
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
        leftAltBoundaryServer_->registerPath(pathFrame_, standbyLeftBound_);
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
    } else if (param.get_name() == "alt_right_bound_file_name") {
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
        rightAltBoundaryServer_->registerPath(pathFrame_, standbyRightBound_);
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

/*
void PathServerNode::onPathOffsetCmd(const PathOffsetCommand::SharedPtr msg) {
  pathServer_->offsetPath(
      msg->effective_from_distance, msg->effective_until_distance,
      std::clamp(msg->leftwards_offset, minLeftwardsOffset_, maxLeftwardsOffset_));
}

void PathServerNode::onPathSwitchingCmd(const BasestationCommand::SharedPtr msg) {
  // we start on pit trajectory, therefore useAltPath = false will keep us on pit lane.
  // The initial transition from pit to non-pit is not controlled from here. It is controlled by the
  // swapActivePathPrecond (i.e. distance between waypoints). manual override to useAltPath_=true is
  // allowed as long as veh_flag are not asserted (see below).
  useAltPath_ = !(msg->take_pit_trajectory);
}

void PathServerNode::onFlagSignal(const RcToCt::SharedPtr msg) {
  // we start on pit trajectory, therefore useAltPath = false will keep us on pit lane.
  // if either veh_flag 2 or 4 asserted, we will fall back to pit path.
  useAltPath_ &= (msg->veh_flag != 2 && msg->veh_flag != 4);
}
*/

void PathServerNode::pubErrMsg(const std::string& description, const int8_t& level,
                               const int8_t& lifetime) {
  // The error message transmision is not implemented when provided as an isolated package.
  /*
  auto msg = std::make_unique<ErrorReport>();
  msg->module = "Planning";
  msg->origin = this->get_name();
  msg->description = description;
  msg->severity = level;
  msg->lifetime = lifetime;
  errPub_->publish(std::move(msg));
  */
}

}  // namespace path_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_server::PathServerNode)
