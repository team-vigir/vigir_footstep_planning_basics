#include <vigir_footstep_planning_plugins/plugin_aggregators/post_processor.h>



namespace vigir_footstep_planning
{
PostProcessor::PostProcessor()
  : ExtendedPluginAggregator<PostProcessor, PostProcessPlugin>("PostProcessor")
{
}

void PostProcessor::postProcessForward(const State& left_foot, const State& right_foot, State& swing_foot) const
{
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    if (plugin)
      plugin->postProcessStepForward(left_foot, right_foot, swing_foot);
  }
}

void PostProcessor::postProcessBackward(const State& left_foot, const State& right_foot, State& swing_foot) const
{
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    if (plugin)
      plugin->postProcessStepBackward(left_foot, right_foot, swing_foot);
  }
}

void PostProcessor::postProcess(msgs::StepPlan step_plan) const
{
  for (PostProcessPlugin::Ptr plugin : getPlugins())
  {
    if (plugin)
      plugin->postProcess(step_plan);
  }
}
}
