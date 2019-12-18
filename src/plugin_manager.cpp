#include <ros/ros.h>
#include <rviz/display_context.h>
#include <rviz/display_group.h>
#include <rviz/failed_display.h>
#include <rviz/yaml_config_reader.h>
#include <rviz/yaml_config_writer.h>
#include <rviz/config.h>

#include "rviz_plugin_manager/plugin_manager.h"
#include "rviz_plugin_manager/PluginLoad.h"
#include "rviz_plugin_manager/PluginUnload.h"
#include "rviz_plugin_manager/PluginGetConfig.h"
#include "rviz_plugin_manager/PluginSetConfig.h"

#include <map>
#include <sstream>
#include <typeinfo>

using namespace rviz_plugin_manager;

PluginManager::PluginManager() : plugin_uid_(0)
{
}

PluginManager::~PluginManager()
{
}


void PluginManager::onEnable()
{
  // Advertise the services
	try
	{
		service_load_ = nh_.advertiseService("rviz_plugin_load", &PluginManager::pluginLoadCallback, this);
		service_unload_ = nh_.advertiseService("rviz_plugin_unload", &PluginManager::pluginUnloadCallback, this);
		service_get_config_ = nh_.advertiseService("rviz_plugin_get_config", &PluginManager::pluginGetConfigCallback, this);
		service_set_config_ = nh_.advertiseService("rviz_plugin_set_config", &PluginManager::pluginSetConfigCallback, this);
		ROS_INFO("PluginManager is now advertising services");
	}
	catch (ros::Exception& e)
	{
		ROS_ERROR("PluginManager couldn't advertise service: %s", e.what());
	}
}

void PluginManager::onDisable()
{
  // Shut down the services. 
	ROS_INFO("PluginManager is shutting down it's services");
	service_load_.shutdown();
	service_unload_.shutdown();
	service_get_config_.shutdown();
	service_set_config_.shutdown();
	display_map_.clear();
}


bool PluginManager::getDistplayByUid(rviz::Display* &disp, long plugin_uid)
{
	disp = NULL;

	// Search for uid from our display map.
	std::map<long, Display*>::iterator disp_it = display_map_.find(plugin_uid);
	if(disp_it == display_map_.end())
	{
		return false;
	}


	// Found display from out map, now let's verify that the display exists in rviz display group.
	rviz::DisplayGroup* disp_group = context_->getRootDisplayGroup();
	for(int i=0;i<disp_group->numDisplays();i++)
	{
		if(disp_group->getDisplayAt(i) == disp_it->second)
		{
			// Display exist, all good.
			disp = disp_it->second;
			return true;
		}

	}
	
	// Someone has removed the display from rviz, remove from our map as well.
	display_map_.erase(disp_it);
	return false;
}


bool PluginManager::pluginLoadCallback(PluginLoad::Request &req, PluginLoad::Response &res)
{
	rviz::DisplayGroup* disp_group = context_->getRootDisplayGroup();
	Display* disp = disp_group->createDisplay(req.plugin_class.c_str());

	try
	{
		if(dynamic_cast<rviz::FailedDisplay*>(disp) != NULL)
		{
			// RViz created Failed Display, it means that the plugin was not loaded.
			// Remove Failed Display and notify
			disp->disconnect();
			disp->deleteLater();
			res.plugin_uid = -1;
			res.code = -1;
			std::stringstream ss;
			ss << "PluginManager failed to load plugin for class '" << req.plugin_class << "'.";
			res.message = ss.str();
			ROS_ERROR_STREAM(res.message); 
		}
		else
		{

			disp->initialize(context_);
      // if config was provided, parse it into rviz config map
      if (req.plugin_config != "")
      {
        rviz::Config config;
        rviz::YamlConfigReader reader;
        reader.readString(config, req.plugin_config.c_str(), "");
        disp->load(config);
      }
      disp->setName(req.plugin_name.c_str());
      disp->setTopic(req.plugin_topic.c_str(), req.plugin_data_type.c_str());
			disp->setEnabled(true);

			disp_group->addDisplay(disp);
			display_map_[plugin_uid_] = disp; // add to map
			res.plugin_uid = plugin_uid_++; // return id and increase for next plugin
			res.code = 0;
		}
	}
	catch (const std::bad_cast& e){
		ROS_INFO("What a heck, PluginManager had bad cast exception: %s", e.what());
	}
	
	return true;
}


bool PluginManager::pluginUnloadCallback(PluginUnload::Request &req, PluginUnload::Response &res)
{
	rviz::Display* disp;
	if(getDistplayByUid(disp, req.plugin_uid))
	{
		disp->disconnect();
		disp->deleteLater();
		res.code = 0;
		res.message = "PluginManager unloaded plugin with UID: " + std::to_string(req.plugin_uid);
		ROS_INFO_STREAM(res.message.c_str()); 
	}
	else
	{
		res.code = -1;
		res.message = "PluginManager didn't find plugin with UID: " + std::to_string(req.plugin_uid);
		ROS_ERROR_STREAM(res.message); 
	}

	return true;
}


bool PluginManager::pluginGetConfigCallback(PluginGetConfig::Request &req, PluginGetConfig::Response &res)
{
	rviz::Display* disp;
	if(getDistplayByUid(disp, req.plugin_uid))
	{
		rviz::Config config;
		disp->save(config);

		// We have to use the writeString() function since writeConfigNode() [that we actually need]
		// is declared private in 'rviz/yaml_config_writer.h'.
		rviz::YamlConfigWriter writer;
		QString filename = "";
		res.config = writer.writeString(config, filename).toStdString();
		
		res.code = 0;
		res.message = "PluginManager successfully returned configuration for plugin with UID:" + std::to_string(req.plugin_uid);
		ROS_INFO_STREAM(res.message); 
	}
	else
	{
		res.code = -1;
		res.message = "PluginManager didn't find plugin with UID:" + std::to_string(req.plugin_uid);
		ROS_ERROR_STREAM(res.message); 
	}

	return true;
}


bool PluginManager::pluginSetConfigCallback(PluginSetConfig::Request &req, PluginSetConfig::Response &res)
{
	rviz::Display* disp;
	if(getDistplayByUid(disp, req.plugin_uid))
	{
		rviz::Config config;
		rviz::YamlConfigReader reader;
		reader.readString(config, req.config.c_str(), ""); // try to parse the config str into rviz config map 
		disp->load(config); // save config to display

		ROS_DEBUG("Got display configuration: \n%s", req.config.c_str());

		res.code = 0;
		res.message = "PluginManager loaded new configuration for display with UID: " + std::to_string(req.plugin_uid);
		ROS_INFO_STREAM(res.message); 
	}
	else
	{
		res.code = -1;
		res.message = "PluginManager didn't find plugin with UID:" + std::to_string(req.plugin_uid);
		ROS_ERROR_STREAM(res.message); 
	}
	return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_manager::PluginManager, rviz::Display)
