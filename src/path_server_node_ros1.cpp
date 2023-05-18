#include "path_server/path_server_node_ros1.hpp"

#include <chrono>
#include <ctime>

namespace path_server {

PathServerNode::PathServerNode(ros::NodeHandle* nh)
    : node_(nh) {
  // path attributes that are subject to change after pathServer_ instantiated
  pathFrame_ = node_->param<std::string>("path_file_reference_frame", "earth");

  // base folder path is only used during initial loading
  std::string pathFolder_ = node_->param<std::string>("path_folder", "");
  if (pathFolder_.length() && pathFolder_.back() != '/') pathFolder_ += "/";

  pathFileName_ = node_->param<std::string>("/path_server/path_file_name", "path.csv");

  lookAheadDist_ = node_->param<double>("/path_server/look_ahead_distance", 100.0);
  lookBehindDist_ = node_->param<double>("look_behind_distance", 0.0);

  isClosedPath_ = node_->param<bool>("is_closed_path", true);
  doInterp_ = node_->param<bool>("perform_interpolation", true);
  interpStep_ = node_->param<double>("interpolation_step_length", 1.0);
  interpretYaw_ = node_->param<bool>("interpret_yaw", true);

  // attributed used to initialize pathServer_ instances only, will not change.
  std::string utmFrame, localEnuFrame, mapFrame, odomFrame, targetFrame, onNewPath;
  targetFrame = node_->param<std::string>("target_reference_frame", "base_link");
  utmFrame = node_->param<std::string>("utm_frame_id", "utm");
  localEnuFrame = node_->param<std::string>("local_enu_frame_id", "local_enu");
  mapFrame = node_->param<std::string>("map_frame_id", "map");
  odomFrame = node_->param<std::string>("odom_frame_id", "odom");

  frontPathTopic_ = node_->param<std::string>("front_path_topic", "front_path");
  rearPathTopic_ = node_->param<std::string>("rear_path_topic", "rear_path");
  this->frontPathPub_ =
      node_->advertise<Path>(frontPathTopic_ + "/path", 10);
  this->rearPathPub_ =
      node_->advertise<Path>(rearPathTopic_ + "/path", 10);

  this->pathCurvaturePub_ =
      node_->advertise<Float32MultiArray>("path_curvature", 10);

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>();
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  geometry_msgs::TransformStamped t;

  // assign header frame ids for path representations
  // cast unique_ptr to raw pointer as tfBuffer_ will continue to exist until after pathServer_ gets
  // destroyed.
  pathServer_ = new PathServer(node_, tfBuffer_.get(), targetFrame, utmFrame, localEnuFrame,
                               mapFrame, odomFrame, onNewPath);
  pathServer_->setPathAttr(lookAheadDist_, lookBehindDist_, isClosedPath_, doInterp_, interpStep_,
                           interpretYaw_, true);
  while (!initialize(pathServer_, pathFolder_ + pathFileName_, pathFrame_))
    ;

  pathPubTimer_ = std::make_unique<ros::Rate>(100.0);
}

std::vector<std::vector<double>> PathServerNode::readPathFromCsv(const std::string& pathFileName) {
  std::vector<std::vector<double>> res;
  std::ifstream pathFileCsv_;
  pathFileCsv_.open(pathFileName.c_str());
  if (!pathFileCsv_.is_open()) {
    ROS_FATAL("PROVIDED PATH FILE NAME IS INVALID. THE NODE WILL NOT OUTPUT ANYTHING.");
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
      ROS_WARN("File %s Line %i is invalid (non-csv or non-numeric) and will be skipped.",
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
  ROS_INFO("Initializing path %s .", pathFileName.c_str());
  std::clock_t tic = std::clock();
  rawCoords = readPathFromCsv(pathFileName);
  // handle error
  if (!rawCoords.size()) {
    ROS_FATAL("Reading path file %s FAILED! Please check file existance and formatting.",
                 pathFileName.c_str());
    pubErrMsg("Reading path file FAILED! Please check file existance and formatting.", 5, 100);
    ros::Duration(1.0).sleep();
    return false;
  }
  try {
    ps->registerPath(pathFrame, rawCoords);
  } catch (std::runtime_error& e) {
    ROS_FATAL("Registration of path FAILED! The specified path frame is invalid or not trasformable.");
    pubErrMsg(
        "Registration of path FAILED! The specified path frame is invalid or not trasformable.", 5,
        15);
    return false;
  }
  ROS_INFO("Path %s registered with %i points.", pathFileName.c_str(),
              static_cast<int>(rawCoords.size()));

  // initialization process guarantees a non-negative currentInd_
  while (node_->ok() && ps->getCurrentInd() < 0) {
    // initialization of frenet frame and register path
    ps->initialize();
    ros::Duration(0.1).sleep();
  }

  double toc = (std::clock() - tic) / (double)CLOCKS_PER_SEC;
  ROS_INFO("Initialized path %s with length %f meters in %f seconds.",
              pathFileName.c_str(), ps->getPathCumulativeDist(), toc);
  return true;
}

/**
 * publish path in target_frame, extracting the segment within
 * [lookBehind, lookAhead]
 */
void PathServerNode::run() {
  while (node_->ok()){
    bool updatedPath = pathServer_->updateStep();
    if (!updatedPath) {
      pubErrMsg("Path server failed to update! Please check path and frame validity!", 4, 1);
    }

    auto frontPath = std::make_unique<Path>(pathServer_->getFrontPath());
    // error handling for empty path
    if (!frontPath->poses.size()) {
      pubErrMsg("Path has run out! <error code 132352>", 5, 1);
    }
    frontPathPub_.publish(*frontPath.get());
    rearPathPub_.publish(pathServer_->getRearPath());

    // publish path curvature
    auto curvatureMsg = std::make_unique<Float32MultiArray>();
    curvatureMsg->data = pathServer_->getFrontCurvature();
    pathCurvaturePub_.publish(*curvatureMsg.get());
    pathPubTimer_->sleep();
  }
}

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

int main(int argc, char** argv){
  ros::init(argc, argv, "path_server");
  ros::NodeHandle n;
  path_server::PathServerNode pathServerInst(&n);
  pathServerInst.run();
  return 0;
}

