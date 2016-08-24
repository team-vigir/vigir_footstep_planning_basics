#include <vigir_footstep_planning_plugins/plugins/state_generator_plugin.h>



namespace vigir_footstep_planning
{
StateGeneratorPlugin::StateGeneratorPlugin(const std::string& name)
  : Plugin(name)
{
}

StateGeneratorPlugin::~StateGeneratorPlugin()
{
}

bool StateGeneratorPlugin::isUnique() const
{
  return false;
}

}
