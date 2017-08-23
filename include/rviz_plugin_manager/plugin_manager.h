#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rviz/display.h>
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginGetConfig.h"
#include "rviz_plugin_manager/PluginSetConfig.h"
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
			bool pluginGetConfigCallback(PluginGetConfig::Request &req, PluginGetConfig::Response &res);
			bool pluginSetConfigCallback(PluginSetConfig::Request &req, PluginSetConfig::Response &res);

		private:
			bool getDistplayByUid(rviz::Display* &disp, long plugin_uid);

			std::map<long,rviz::Display*> display_map_;
			ros::NodeHandle nh_;
			ros::ServiceServer service_load_;
			ros::ServiceServer service_unload_;
			ros::ServiceServer service_get_config_;
			ros::ServiceServer service_set_config_;
			long plugin_uid_;
	};
}
