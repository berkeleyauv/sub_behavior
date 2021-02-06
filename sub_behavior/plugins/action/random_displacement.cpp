#include <random>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/vector3.hpp"

using namespace BT;

/**
* Action that sets puts a random displacement on the blackboard for other nodes to use
*/
class RandomDisplacementNode : public SyncActionNode
{
public:
  RandomDisplacementNode(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config), distribution(-3.0,3.0){}

  static PortsList providedPorts()
  {
    return { OutputPort<geometry_msgs::msg::Vector3>("displacement") };
  }

private:
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution;

  virtual BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Vector3 displacement = geometry_msgs::msg::Vector3();
    displacement.x = distribution(generator);
    displacement.y = distribution(generator);
    displacement.z = distribution(generator);
    setOutput("displacement", displacement );
    return NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<RandomDisplacementNode>("RandomDisplacement");
}