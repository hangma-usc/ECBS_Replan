#pragma once

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
    double x;
    double y;
    double z;
    orientation_t orientation;
    int locationId;
    action_t action;
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
