// filter_component: rclcpp_components-registered composable node. Sub
// "points_raw" -> pub "points_filtered", passthrough (both relative,
// resolved under the container's namespace, e.g. /perception/...).
//
// Loaded into the container declared in launch/bringup.launch.xml,
// exercising the composable scheduling path: isolated mode gives it its
// own process (fork+exec'd component_node), scheduled on its LOADED event.

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace rt_demo
{

class FilterComponent : public rclcpp::Node
{
public:
  explicit FilterComponent(const rclcpp::NodeOptions & options)
  : Node("filter_component", options)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("points_filtered", 10);

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "points_raw", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        auto out = std_msgs::msg::String();
        out.data = msg->data;
        pub_->publish(out);
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace rt_demo

RCLCPP_COMPONENTS_REGISTER_NODE(rt_demo::FilterComponent)
