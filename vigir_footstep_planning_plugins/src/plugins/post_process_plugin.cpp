#include <vigir_footstep_planning_plugins/plugins/post_process_plugin.h>



namespace vigir_footstep_planning
{
PostProcessPlugin::PostProcessPlugin(const std::string& name)
  : Plugin(name)
{
}

bool PostProcessPlugin::isUnique() const
{
  return false;
}

void PostProcessPlugin::postProcessStepForward(const State& /*left_foot*/, const State& /*right_foot*/, State& /*swing_foot*/) const
{
}

void PostProcessPlugin::postProcessStepBackward(const State& /*left_foot*/, const State& /*right_foot*/, State& /*swing_foot*/) const
{
}

void PostProcessPlugin::postProcessStep(const msgs::Step& /*left_foot*/, const msgs::Step& /*right_foot*/, msgs::Step& /*swing_foot*/, msgs::StepPlan& /*step_plan*/) const
{
}

void PostProcessPlugin::postProcess(msgs::StepPlan step_plan) const
{
  if (step_plan.steps.empty())
    return;

  std::vector<msgs::Step>::iterator itr = step_plan.steps.begin();

  // determine start configuration
  msgs::Step left_foot, right_foot = *itr;

  if (left_foot.foot.foot_index != msgs::Foot::LEFT)
    left_foot.foot = step_plan.start.left;

  if (right_foot.foot.foot_index != msgs::Foot::RIGHT)
    right_foot.foot = step_plan.start.right;

  // iterate step plan
  for (; itr != step_plan.steps.end(); itr++)
  {
    msgs::Step& swing_foot = *itr;

    postProcessStep(left_foot, right_foot, swing_foot, step_plan);

    if (swing_foot.foot.foot_index == msgs::Foot::LEFT)
      left_foot = swing_foot;
    else
      right_foot = swing_foot;
  }
}
}
