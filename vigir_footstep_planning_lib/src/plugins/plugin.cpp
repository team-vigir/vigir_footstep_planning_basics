#include <vigir_footstep_planning_lib/plugins/plugin.h>

namespace vigir_footstep_planning
{
Plugin::Plugin(const std::string& name, const std::string& type_id, const ParameterSet& params)
  : name(name)
  , type_id(type_id)
{
  loadParams(params);
}

Plugin::Plugin(const std::string& name, const std::string& type_id)
  : Plugin(name, type_id, ParameterManager::getActive())
{
}

Plugin::~Plugin()
{
}

const std::string& Plugin::getName() const
{
  return name;
}

const std::string& Plugin::getTypeId() const
{
  return type_id;
}

bool Plugin::isUnique() const
{
  return true;
}
}
