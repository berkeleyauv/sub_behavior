#include "sub_behavior_interfaces/action/move_distance.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtAction.hpp"

#include "geometry_msgs/msg/vector3.hpp"

using namespace BT;
using MoveDistanceAction = sub_behavior_interfaces::action::MoveDistance;

class MoveDistanceNode : public BtAction<MoveDistanceAction>
{
public:
  MoveDistanceNode(const std::string & name, const BT::NodeConfiguration & config)
    : BtAction<MoveDistanceAction>(name, config) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<geometry_msgs::msg::Vector3>("distances")});
  }

  MoveDistanceAction::Goal populate_goal() override
  {
    MoveDistanceAction::Goal goal;
    goal.distances = *getInput<geometry_msgs::msg::Vector3>("distances");
    return goal;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<MoveDistanceNode>("MoveDistanceNode");
}