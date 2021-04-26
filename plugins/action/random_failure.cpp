#include <random>
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

/**
* Simple actions that returns FAILURE with p probability.
*/
class RandomFailureNode : public SyncActionNode
{
public:
  RandomFailureNode(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config), distribution(0.0,1.0){}

  static PortsList providedPorts()
  {
    return { InputPort<double>("probability") };
  }

private:
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution;

  virtual BT::NodeStatus tick() override
  {
    double p;
    getInput<double>("probability", p);
    if(distribution(generator) < p){
      return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<RandomFailureNode>("RandomFailure");
}