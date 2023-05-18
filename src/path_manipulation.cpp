#include "path_server/path_server.hpp"

namespace path_server {

void PathServer::offsetPath(const float& from, const float& to, const float& leftingAmount) {
  // TODO: consider timing delays from header
  existingOffset_ = getCurrentOffset();
  doOffset_ = existingOffset_ != 0. || leftingAmount != 0.;
  pathOffsetOdomMark_ = odometer_;
  // Limit lateral rate: the lane change is completed in no less than
  // delta/pathOffsetEaseInRate_ distance ahead, using linear lateral interp.
  float delta = leftingAmount - existingOffset_;
  pathOffsetEaseInDist_ = std::fmax(from, std::fabs(delta / pathOffsetEaseInRate_));
  pathOffsetOdomSet_ = odometer_ + static_cast<double>(pathOffsetEaseInDist_);
  pathOffsetOdomLift_ = odometer_ + static_cast<double>(std::fmax(pathOffsetEaseInDist_, to));
  // add a small number to prevent divide by zero
  pathOffsetOdomClear_ =
      pathOffsetOdomLift_ + std::fabs(leftingAmount / pathOffsetEaseInRate_) + 0.001;
  pathOffsetToLeft_ = leftingAmount;
}

void PathServer::appendPoses(std::vector<PoseStamped>& target,
                             const std::vector<PoseStamped>& input) {
  target.insert(target.end(), input.begin(), input.end());
  // re-calculate frenet-frame distances and curvature
  initFrenetXDistance();
  initCurvature();
}

void PathServer::replacePoses(std::vector<PoseStamped>& target,
                              const std::vector<PoseStamped>& input) {
  // re-initialization requires path_frame -> base_link transfomation available.
  target = input;
  // reset initialization status
  currentInd_ = -1;
  initialize();
}

// TODO: more path switching approaches.
// void PathServer::intersectPoses(std::vector<PoseStamped> &target, const std::vector<PoseStamped>
// &input){}

}  // namespace path_server
