#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rviz/display_context.h>
#include <rviz/display_group.h>
#include <map>
#include "rviz_plugin_manager/plugin_manager.h"
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginProperty.h"

using namespace rviz_plugin_manager;

PluginManager::PluginManager() : plugin_uid_(0)
{
	ROS_INFO("PluginManager created.");
}

PluginManager::~PluginManager()
{
	ROS_INFO("PluginManager destroyed.");
}


void PluginManager::onEnable()
{
	ROS_INFO("PluginManager enabled");
	try
	{
		service_load_ = nh_.advertiseService("rviz_plugin_load", &PluginManager::pluginLoadCallback, this);
		service_unload_ = nh_.advertiseService("rviz_plugin_unload", &PluginManager::pluginUnloadCallback, this);
		service_property_ = nh_.advertiseService("rviz_plugin_set_property", &PluginManager::pluginPropertyCallback, this);
		ROS_INFO("PluginManager is now advertising services");
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR("Couldn't advertise service: %s", e.what());
	}
}

void PluginManager::onDisable()
{
	ROS_INFO("Shutting down PluginManager services");
	service_load_.shutdown();
	service_unload_.shutdown();
	service_property_.shutdown();
	display_map_.clear();
}


bool PluginManager::pluginLoadCallback(PluginLoad::Request &req, PluginLoad::Response &res)
{
	
	ROS_INFO("Got request for loading plugin: \n\tplugin_name: %s\n\ttopic: %s\n\tplugin_uid: %ld", 
			req.plugin_name.c_str(), req.plugin_topic.c_str(), plugin_uid_);

	rviz::DisplayGroup* disp_group= context_->getRootDisplayGroup();
	Display* disp = disp_group->createDisplay(req.plugin_class.c_str());
	disp->initialize(context_);
	disp->setName(req.plugin_name.c_str());
	disp->setTopic(req.plugin_topic.c_str(), req.plugin_datatype.c_str());
	disp->setEnabled(true);
	disp_group->addDisplay(disp);
	display_map_[plugin_uid_] = disp; // add to map
	res.plugin_uid = plugin_uid_++; // return id and increase for next plugin
	return true;
}


bool PluginManager::pluginUnloadCallback(PluginUnload::Request &req, PluginUnload::Response &res)
{
	
	ROS_INFO("Got request for unloading plugin: \n\tplugin_uid: %ld", 
			req.plugin_uid);

	std::map<long, Display*>::iterator disp_it = display_map_.find(req.plugin_uid);
	if(disp_it == display_map_.end())
	{
		ROS_ERROR("Plugin with id: %ld was not found (already removed?)", req.plugin_uid); 
		res.code = -1;
	}
	else
	{
		Display* disp = disp_it->second;
		disp->disconnect();
		disp->deleteLater();
		res.code = 0;
	}
	return true;
}


bool PluginManager::pluginPropertyCallback(PluginProperty::Request &req, PluginProperty::Response &res)
{
	return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_manager::PluginManager, rviz::Display)
