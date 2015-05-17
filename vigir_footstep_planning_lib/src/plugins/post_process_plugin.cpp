#include <vigir_footstep_planning_lib/plugins/post_process_plugin.h>

namespace vigir_footstep_planning
{
PostProcessPlugin::PostProcessPlugin(const std::string& name, const ParameterSet& params)
  : Plugin("post_processor", "post_process_plugin", params)
{
}

PostProcessPlugin::PostProcessPlugin(const std::string& name)
  : Plugin("post_processor", "post_process_plugin")
{
}

bool PostProcessPlugin::isUnique() const
{
  return false;
}

void PostProcessPlugin::postProcessStepForward(const State& /*left*/, const State& /*right*/, State& /*swing*/) const
{
}

void PostProcessPlugin::postProcessStepBackward(const State& /*left*/, const State& /*right*/, State& /*swing*/) const
{
}

void PostProcessPlugin::postProcessStep(const msgs::Step& /*left*/, const msgs::Step& /*right*/, msgs::Step& /*swing*/, msgs::StepPlan& /*step_plan*/) const
{
}

void PostProcessPlugin::postProcess(msgs::StepPlan step_plan) const
{
  if (step_plan.steps.empty())
    return;

  std::vector<msgs::Step>::iterator itr = step_plan.steps.begin();

  // determine start configuration
  msgs::Step left, right = *itr;

  if (left.foot.foot_index != msgs::Foot::LEFT)
    left.foot = step_plan.start.left;

  if (right.foot.foot_index != msgs::Foot::RIGHT)
    right.foot = step_plan.start.right;

  // iterate step plan
  for (; itr != step_plan.steps.end(); itr++)
  {
    msgs::Step& swing = *itr;

    postProcessStep(left, right, swing, step_plan);

    if (swing.foot.foot_index == msgs::Foot::LEFT)
      left = swing;
    else
      right = swing;
  }
}
}
