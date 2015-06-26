#include <vigir_footstep_planning_lib/plugins/plugin_manager.h>

namespace vigir_footstep_planning
{
PluginManager::Ptr PluginManager::singelton = PluginManager::Ptr();

PluginManager::PluginManager()
  : reachability_loader("vigir_footstep_planner", "vigir_footstep_planning::ReachabilityPlugin")
  , step_cost_estimator_loader("vigir_footstep_planner", "vigir_footstep_planning::StepCostEstimatorPlugin")
  , heuristic_loader("vigir_footstep_planner", "vigir_footstep_planning::HeuristicPlugin")
  , post_process_loader("vigir_footstep_planner", "vigir_footstep_planning::PostProcessPlugin")
{
}

PluginManager::Ptr& PluginManager::Instance()
{
   if (!singelton)
      singelton.reset(new PluginManager());
   return singelton;
}

void PluginManager::addPlugin(Plugin::Ptr plugin)
{
  if (!plugin)
  {
    ROS_ERROR("[PluginManager] Got NULL pointer as plugin. Fix it immediatly!");
    return;
  }

  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.find(plugin->getName());
  Plugin::Ptr unique_plugin;

  if (itr != Instance()->plugins_by_name.end()) // replace by name
    ROS_INFO("[PluginManager] Plugin '%s' with type_id '%s' is replaced by '%s' with type_id '%s'!", itr->second->getName().c_str(), itr->second->getTypeId().c_str(), plugin->getName().c_str(), plugin->getTypeId().c_str());
  else if (plugin->isUnique() && getUniquePluginByTypeId(plugin->getTypeId(), unique_plugin)) // replace by uniqueness
  {
    ROS_INFO("[PluginManager] Unique plugin '%s' with type_id '%s' is replaced by '%s'!", unique_plugin->getName().c_str(), unique_plugin->getTypeId().c_str(), plugin->getName().c_str());
    Instance()->plugins_by_name.erase(Instance()->plugins_by_name.find(unique_plugin->getName())); // prevent outputs by removePlugin call
  }
  else
    ROS_INFO("[PluginManager] Added new plugin '%s' with type_id '%s'", plugin->getName().c_str(), plugin->getTypeId().c_str());

  Instance()->plugins_by_name[plugin->getName()] = plugin;
}

void PluginManager::addPlugin(Plugin* plugin)
{
  Plugin::Ptr plugin_ptr(plugin);
  addPlugin(plugin_ptr);
}

bool PluginManager::addPlugin(const std::string type)
{
  boost::shared_ptr<vigir_footstep_planning::Plugin> p;

  try
  {
    if (Instance()->reachability_loader.isClassAvailable(type))
      p = Instance()->reachability_loader.createInstance(type);
    else if (Instance()->step_cost_estimator_loader.isClassAvailable(type))
      p = Instance()->step_cost_estimator_loader.createInstance(type);
    else if (Instance()->heuristic_loader.isClassAvailable(type))
      p = Instance()->heuristic_loader.createInstance(type);
    else if (Instance()->post_process_loader.isClassAvailable(type))
      p = Instance()->post_process_loader.createInstance(type);
    else
    {
      ROS_ERROR("[PluginManager] Plugin of class '%s' is unknown!", type.c_str());
      return false;
    }
  }
  catch(pluginlib::PluginlibException& e)
  {
    ROS_ERROR("[PluginManager] The plugin failed to load for some reason. Error: %s", e.what());
    return false;
  }

  PluginManager::addPlugin(p);
  return true;
}

bool PluginManager::getPluginByName(const std::string& name, Plugin::Ptr& plugin)
{
  plugin.reset();

  std::map<std::string, Plugin::Ptr>::const_iterator itr = Instance()->plugins_by_name.find(name);
  if (itr == Instance()->plugins_by_name.end())
    return false;

  plugin = itr->second;
  return true;
}

bool PluginManager::getPluginsByTypeId(const std::string& type_id, std::vector<Plugin::Ptr>& plugins)
{
  plugins.clear();

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (itr->second->getTypeId() == type_id)
      plugins.push_back(itr->second);
  }

  return !plugins.empty();
}

bool PluginManager::getUniquePluginByTypeId(const std::string& type_id, Plugin::Ptr& plugin)
{
  plugin.reset();

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (itr->second->isUnique() && itr->second->getTypeId() == type_id)
    {
      plugin = itr->second;
      return true;
    }
  }

  return false;
}

void PluginManager::removePlugin(Plugin::Ptr& plugin)
{
  removePluginByName(plugin->getName());
}

void PluginManager::removePluginByName(const std::string& name)
{
  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.find(name);
  if (itr == Instance()->plugins_by_name.end())
    return;

  ROS_INFO("[PluginManager] Removed plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
  Instance()->plugins_by_name.erase(itr);
}

void PluginManager::removePluginsByTypeId(const std::string& type_id)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end();)
  {
    if (itr->second->getTypeId() == type_id)
    {
      ROS_INFO("[PluginManager] Removed plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
      Instance()->plugins_by_name.erase(itr++);
    }
    else
      itr++;
  }
}

bool PluginManager::hasPlugin(Plugin::Ptr& plugin)
{
  std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.find(plugin->getName());
  return (itr != Instance()->plugins_by_name.end() && itr->second->getTypeId() == plugin->getTypeId());
}

bool PluginManager::hasPluginByName(const std::string& name)
{
  return Instance()->plugins_by_name.find(name) != Instance()->plugins_by_name.end();
}

bool PluginManager::hasPluginsByTypeId(const std::string& type_id)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (itr->second->getTypeId() == type_id)
      return true;
  }
}

void PluginManager::loadParams(const ParameterSet& params)
{
  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
    itr->second->loadParams(params);
}

bool PluginManager::initializePlugins(ros::NodeHandle& nh, const ParameterSet& params)
{
  bool result = true;

  for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
  {
    if (!itr->second->initialize(nh, params))
    {
      result = false;
      ROS_ERROR("[PluginManager] Failed to initialize plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
    }
  }

  return result;
}

bool PluginManager::initializePlugins(ros::NodeHandle& nh)
{
  return initializePlugins(nh, ParameterManager::getActive());
}
}
