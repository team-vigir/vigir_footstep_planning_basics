//=================================================================================================
// Copyright (c) 2015, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef VIGIR_FOOTSTEP_PLANNING_LIB_PLUGIN_MANAGER_H__
#define VIGIR_FOOTSTEP_PLANNING_LIB_PLUGIN_MANAGER_H__

#include <ros/ros.h>

#include <boost/noncopyable.hpp>

#include <vigir_footstep_planning_msgs/ParameterSet.h>

#include <vigir_footstep_planning_lib/plugins/plugin.h>



namespace vigir_footstep_planning
{
class PluginManager
  : boost::noncopyable
{
public:
  template<typename T>
  static void addPlugin()
  {
    Plugin::Ptr plugin(new T());
    addPlugin(plugin);
  }
  static void addPlugin(Plugin::Ptr& plugin);
  static void addPlugin(Plugin* plugin); // this function takes over pointer and will free memory automatically, when plugin is removed

  /**
   * Returns first found plugin matching typename T. If specific element should be returned, set name.
   */
  template<typename T>
  static bool getPlugin(boost::shared_ptr<T>& plugin, const std::string& name = std::string())
  {
    plugin.reset();

    // name specific search
    if (name.size())
    {
      Plugin::Ptr p;
      if (getPluginByName(name, p))
      {
        plugin = boost::dynamic_pointer_cast<T>(p);
        if (plugin)
          return true;
      }

      ROS_ERROR("[PluginManager] Couldn't find any matching plugin named '%s'!", name.c_str());
      return false;
    }
    // type specific search
    else
    {
      std::vector<boost::shared_ptr<T> > plugins;
      getPluginsByType(plugins);

      for (typename std::vector<boost::shared_ptr<T> >::iterator itr = plugins.begin(); itr != plugins.end(); itr++)
      {
        plugin = boost::dynamic_pointer_cast<T>(*itr);
        if (plugin)
          return true;
      }

      ROS_ERROR("[PluginManager] Couldn't find any matching plugin!");
      return false;
    }
    return false;
  }
  static bool getPluginByName(const std::string& name, Plugin::Ptr& plugin);

  /// return all plugins derived by class T in alphabetical order (name)
  template<typename T>
  static bool getPluginsByType(std::vector<boost::shared_ptr<T> >& plugins)
  {
    plugins.clear();

    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
        plugins.push_back(plugin);
    }

    return !plugins.empty();
  }
  static bool getPluginsByTypeId(const std::string& type_id, std::vector<Plugin::Ptr>& plugins);

  /// returns a plugin marked as unique of specific type id
  static bool getUniquePluginByTypeId(const std::string& type_id, Plugin::Ptr& plugin);

  static void removePlugin(Plugin::Ptr& plugin);

  static void removePluginByName(const std::string& name);

  template<typename T>
  static void removePluginsByType()
  {
    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end();)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
      {
        ROS_INFO("[PluginManager] Removed plugin '%s' with type_id '%s'", itr->second->getName().c_str(), itr->second->getTypeId().c_str());
        Instance()->plugins_by_name.erase(itr++);
      }
      else
        itr++;
    }
  }
  static void removePluginsByTypeId(const std::string& type_id);

  static bool hasPlugin(Plugin::Ptr& plugin);

  static bool hasPluginByName(const std::string& name);

  template<typename T>
  static bool hasPluginsByType()
  {
    for (std::map<std::string, Plugin::Ptr>::iterator itr = Instance()->plugins_by_name.begin(); itr != Instance()->plugins_by_name.end(); itr++)
    {
      boost::shared_ptr<T> plugin = boost::dynamic_pointer_cast<T>(itr->second);
      if (plugin)
        return true;
    }
    return false;
  }
  static bool hasPluginsByTypeId(const std::string& type_id);

  static void loadParams(const ParameterSet& params);

  // typedefs
  typedef boost::shared_ptr<PluginManager> Ptr;
  typedef boost::shared_ptr<const PluginManager> ConstPtr;

protected:
  PluginManager();

  static PluginManager::Ptr& Instance();

  static PluginManager::Ptr singelton;

  std::map<std::string, Plugin::Ptr> plugins_by_name;
};
}

#endif
