#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rviz/display.h>
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginProperty.h"
#include <vector>

#include <QObject>

namespace rviz_plugin_manager
{
	class PluginManager: public rviz::Display
	{
		Q_OBJECT
		public:
			PluginManager();
			virtual ~PluginManager();

		public Q_SLOTS:
			virtual void onEnable();
			virtual void onDisable();

		public:
			bool pluginLoadCallback(PluginLoad::Request &req, PluginLoad::Response &res);
			bool pluginUnloadCallback(PluginUnload::Request &req, PluginUnload::Response &res);
			bool pluginPropertyCallback(PluginProperty::Request &req, PluginProperty::Response &res);

		private:
			std::map<long,rviz::Display*> display_map_;
			ros::NodeHandle nh_;
			ros::ServiceServer service_load_;
			ros::ServiceServer service_unload_;
			ros::ServiceServer service_property_;
			long plugin_uid_;
	};
}
