#ifndef _PATH_SERVER__PATH_SERVER_NODE_HPP_
#define _PATH_SERVER__PATH_SERVER_NODE_HPP_

// #define DEBUG

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_server/path_server.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using std_msgs::msg::Bool;
using std_msgs::msg::Float32MultiArray;

namespace path_server {
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
  bool initialize(PathServer* ps, const std::string& pathFileName, const std::string& pathFrame);

 private:
  std::vector<std::vector<double>> readPathFromCsv(const std::string& pathFileName);

  void onPathPublish();

  // void onPathOffsetCmd(const PathOffsetCommand::SharedPtr msg);

  // void onPathSwitchingCmd(const BasestationCommand::SharedPtr msg);

  // void onFlagSignal(const RcToCt::SharedPtr msg);

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

  double lookAheadDist_, lookBehindDist_, interpStep_;
  double pathOffsetRateLimit_;
  bool isClosedPath_, interpretYaw_, doInterp_, doOffset_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::string frontPathTopic_, rearPathTopic_;
  rclcpp::TimerBase::SharedPtr pathPubTimer_{nullptr};

  /**
   * @brief the center, left and right paths all support hot-swapping.
   * set the new parameter for *Alt* path file names, and toggle the path switching boolean.
   */
  std::string pathFrame_, activePathFileName_, altPathFileName_, leftBoundFileName_,
      leftAltBoundFileName_, rightBoundFileName_, rightAltBoundFileName_;
  std::vector<std::vector<double>> standbyPath_, standbyLeftBound_, standbyRightBound_;
  bool useAltPath_ = false;
  bool onAltPath_ = false;

  /**
   * @brief In IAC, we typically start in pit lane and switch between pit and race track.
   * Therefore, !onAltPath indicates whether we are in pit lane.
   */
  rclcpp::Publisher<Bool>::SharedPtr inPitLanePub_;

  float minLeftwardsOffset_ = 0., maxLeftwardsOffset_ = 0.;

  rclcpp::Publisher<Float32MultiArray>::SharedPtr offsetRangePub_, pathCurvaturePub_,
      offsetDistPub_;

  // rclcpp::Publisher<ErrorReport>::SharedPtr errPub_;

  rclcpp::Publisher<Path>::SharedPtr frontPathPub_, rearPathPub_, frontOffsetPathPub_,
      rearOffsetPathPub_, leftBoundaryPub_, rightBoundaryPub_;
#ifdef DEBUG
  rclcpp::Publisher<Path>::SharedPtr frontAltPathPub_;
#endif

  // rclcpp::Subscription<PathOffsetCommand>::SharedPtr pathOffsetSub_;
  // rclcpp::Subscription<BasestationCommand>::SharedPtr pathSwitchingSub_;
  // rclcpp::Subscription<RcToCt>::SharedPtr flagSigSub_;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr paramCbHandle_;

};  // class

}  // namespace path_server

#endif  // _PATH_SERVER__PATH_SERVER_HPP_
