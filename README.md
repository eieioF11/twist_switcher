# twist_switcher
Node that performs switching of twist, emergency stop, etc.

## Input
- Any geometry_msgs::msg::Twist topic
- twist_bridge/switch : twist switching topic. Send topic name in std_msgs::msg::String to switch.
- twist_bridge/ems : Emergency Stop Service. Output 0 when True in std_srvs::srv::SetBool.
## Output
- geometry_msgs::msg::Twist topic

## Yaml Example
When "auto/cmd_vel" or "manual/cmd_vel" is sent to twist_bridge/switch by std_msgs::msg::String, the value received from the twist topic is output as "/cmd_vel".
```
twist_bridge_node:
  ros__parameters:
    twist_bridge:
      topic_name:
        input_twist: ["auto/cmd_vel", "manual/cmd_vel"]
        output_twist: "/cmd_vel"
```