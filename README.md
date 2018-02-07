# RViz plugin manager

## Introduction
 This RViz plugin enables loading, unloading, and configuring other RViz display plugins dynamically through service calls.

## How it works?
The plugin manager plugin advertises 4 services:
* /rviz\_plugin\_load
* /rviz\_plugin\_unload
* /rviz\_plugin\_get\_config
* /rviz\_plugin\_set\_config

When loading a plugin, an UID is returned. This UID is later used to unload or configure the loaded plugin. Check the .srv files for complete set of parameters.

## Usage
Load the manager plugin in RViz by using the add button or specify it in the .rviz configuration file. If the plugin manager is enabled, you can use the services, for example:

```
rosservice call /rviz_plugin_load "rviz/Marker" "My plugin name" "/marker_topic" ""
rosservice call /rviz_plugin_load "rviz/Image" "My image" "/webcam/image" "std_msgs/Image"
rosservice call /rviz_plugin_get_config 0
rosservice call /rviz_plugin_set_config 0 \"'Enabled: true'\"
rosservice call /rviz_plugin_unload 0
```
