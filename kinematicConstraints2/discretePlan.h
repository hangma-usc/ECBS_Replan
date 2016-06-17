#pragma once

#include <Eigen/Core>

namespace discretePlan
{

  enum action_t
  {
    ACTION_WAIT         = 0,
    ACTION_MOVE         = 1,
    ACTION_ROTATE_LEFT  = 2,
    ACTION_ROTATE_RIGHT = 3,
  };

  enum orientation_t
  {
    ORIENTATION_NORTH = 0,
    ORIENTATION_EAST  = 1,
    ORIENTATION_SOUTH = 2,
    ORIENTATION_WEST  = 3,
  };

  struct state
  {
    Eigen::Vector3d position;
//    double x;
//    double y;
//    double z;
    orientation_t orientation;
    int locationId;
    action_t action;
    std::string name;

    double theta() const {
      double theta = 0;
      switch (orientation) {
        case ORIENTATION_EAST:
          theta = 0;
          break;
        case ORIENTATION_WEST:
          theta = M_PI;
          break;
        case ORIENTATION_NORTH:
          theta = M_PI/2.0f;
          break;
        case ORIENTATION_SOUTH:
          theta = 1.5f * M_PI;
          break;
        default:
          break;
      }
      return theta;
    }

  };

  struct agent
  {
    std::vector<state> states;
    uint32_t group;
    double max_v;
    double max_w;
  };

  struct discretePlan
  {
    std::vector<agent> agents;
  };

} // namespace discretePlan
