#include "path_server/path_server.hpp"

namespace planning {

void PathServer::appendPoses(std::vector<PoseStamped>& target,
                             const std::vector<PoseStamped>& input) {
  target.insert(target.end(), input.cbegin(), input.cend());

  // TODO: this is problematic.
  /*
  std::vector<PoseStamped>* path = &(utmCoords_.poses);
  try {
    path = path_utils::firstAvailable(
        {&(utmCoords_.poses), &(localEnuCoords_.poses), &(mapCoords_.poses), &(odomCoords_.poses)});
  } catch (std::runtime_error& e) {
    path = nullptr;
  }

  // re-calculate frenet-frame distances and curvature
  initFrenetXDistance(path);
  initCurvature(path);
  */
  initialize(true);
}

void PathServer::replacePoses(std::vector<PoseStamped>& target,
                              const std::vector<PoseStamped>& input) {
  // re-initialization requires path_frame -> base_link transfomation available.
  target = input;
  // force re-initialize
  initialize(true);
}

// TODO: more path switching approaches.
// void PathServer::intersectPoses(std::vector<PoseStamped> &target, const std::vector<PoseStamped>
// &input){}

}  // namespace planning
