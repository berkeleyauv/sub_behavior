//
// Created by michael on 2/6/21.
//

#include "sub_interfaces/srv/snap_photo.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behavior_tree/BtService.hpp"

using namespace BT;
using SnapPhotoService = sub_interfaces::srv::SnapPhoto;

class SnapPhotoNode : public BtService<SnapPhotoService>
{
public:
  SnapPhotoNode(const std::string & name, const BT::NodeConfiguration & config)
    : BtService<SnapPhotoService>(name, config) {}

  static PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("path")});
  }

  SnapPhotoService::Request::SharedPtr populate_request() override
  {
    SnapPhotoService::Request::SharedPtr request = std::make_shared<SnapPhotoService::Request>();
    request->image_topic = "test_request";
    return request;
  }

  BT::NodeStatus handle_response(SnapPhotoService::Response::SharedPtr response) override
  {
    RCLCPP_INFO(_node->get_logger(),  "Service call complete: " + response->path);
    return BT::NodeStatus::SUCCESS;
  }
};

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<SnapPhotoNode>("SnapPhotoNode");
}