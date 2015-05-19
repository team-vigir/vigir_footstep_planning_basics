#include <vigir_footstep_planning_lib/parameter_manager.h>

namespace vigir_footstep_planning
{
ParameterManager::Ptr ParameterManager::singelton = ParameterManager::Ptr();

ParameterManager::ParameterManager()
  : active_parameter_set(new ParameterSet())
{
}

void ParameterManager::initTopics(ros::NodeHandle& nh)
{
  // subscribe topics
  Instance()->update_parameter_set_sub = nh.subscribe("params/update_parameter_set", 1, &ParameterManager::updateParameterSet, Instance().get());

  // start own services
  Instance()->set_parameter_set_srv = nh.advertiseService("params/set_parameter_set", &ParameterManager::setParameterSetService, Instance().get());
  Instance()->get_parameter_set_srv = nh.advertiseService("params/get_parameter_set", &ParameterManager::getParameterSetService, Instance().get());
  Instance()->get_all_parameter_sets_srv = nh.advertiseService("params/get_all_parameter_sets", &ParameterManager::getAllParameterSetsService, Instance().get());
  Instance()->get_parameter_set_names_srv = nh.advertiseService("params/get_parameter_set_names", &ParameterManager::getParameterSetNamesService, Instance().get());

  // init action servers
  Instance()->set_parameter_set_as = SimpleActionServer<msgs::SetParameterSetAction>::create(nh, "params/set_parameter_set", true, boost::bind(&ParameterManager::setParameterSetAction, Instance().get(), boost::ref(Instance()->set_parameter_set_as)));
  Instance()->get_parameter_set_as = SimpleActionServer<msgs::GetParameterSetAction>::create(nh, "params/get_parameter_set", true, boost::bind(&ParameterManager::getParameterSetAction, Instance().get(), boost::ref(Instance()->get_parameter_set_as)));
  Instance()->get_all_parameter_sets_as = SimpleActionServer<msgs::GetAllParameterSetsAction>::create(nh, "params/get_all_parameter_sets", true, boost::bind(&ParameterManager::getAllParameterSetsAction, Instance().get(), boost::ref(Instance()->get_all_parameter_sets_as)));
  Instance()->get_parameter_set_names_as = SimpleActionServer<msgs::GetParameterSetNamesAction>::create(nh, "params/get_parameter_set_names", true, boost::bind(&ParameterManager::getParameterSetNamesAction, Instance().get(), boost::ref(Instance()->get_parameter_set_names_as)));
}

ParameterManager::Ptr& ParameterManager::Instance()
{
 if (!singelton)
    singelton.reset(new ParameterManager());
 return singelton;
}

void ParameterManager::clear()
{
  Instance()->param_sets.clear();
  Instance()->active_parameter_set.reset(new ParameterSet());
}

bool ParameterManager::empty()
{
  return Instance()->param_sets.empty();
}

size_t ParameterManager::size()
{
  return Instance()->param_sets.size();
}

bool ParameterManager::loadFromFile(const boost::filesystem::path& path, ParameterSet& params)
{
  // load yaml file into rosparam server
  std::string name_space = ros::this_node::getNamespace() + "/" + path.filename().c_str();
  name_space.resize(name_space.size()-5);
  std::replace(name_space.begin(), name_space.end(), '.', '_');

  std::string cmd = "rosparam load " + std::string(path.c_str()) + " " + std::string(name_space.c_str());
  if (system(cmd.c_str()))
    return false;

  // get parameter as XmlRpcValue
  ros::NodeHandle nh(name_space);
  XmlRpc::XmlRpcValue val;
  nh.getParam("/", val);

  // cleanup
  cmd = "rosparam delete " + std::string(name_space.c_str());
  if (system(cmd.c_str()))
    return false;

  // parse XmlRpcValue
  params.clear();
  return params.fromXmlRpcValue(val);
}

void ParameterManager::loadParameterSets(const std::string& path)
{
  if (!boost::filesystem::exists(path))
  {
    ROS_ERROR("[loadParameterSets] Path not found: '%s'", path.c_str());
    return;
  }

  if (!boost::filesystem::is_directory(path))
  {
    ROS_ERROR("[loadParameterSets] '%s' is not a directory!", path.c_str());
    return;
  }

  // cycle through the directory
  for(boost::filesystem::directory_iterator itr(path); itr != boost::filesystem::directory_iterator(); itr++)
  {
    if (boost::filesystem::is_regular_file(*itr) and itr->path().extension() == ".yaml")
    {
      ParameterSet params;
      if (Instance()->loadFromFile(itr->path(), params))
      {
        if (!hasParameterSet(params.getName()))
        {
          updateParameterSet(params);
          ROS_INFO("[loadParameterSets] Loaded '%s' from '%s' with %u parameters", params.getName().c_str(), itr->path().filename().c_str(), params.size());
        }
        else
          ROS_WARN("[loadParameterSets] Set named '%s' already exist. Parameters from '%s' with %u parameters can't be added.", params.getName().c_str(), itr->path().filename().c_str(), params.size());
      }
      else
        ROS_ERROR("[loadParameterSets] Couldn't load parameters from '%s'", itr->path().filename().c_str());
    }
  }

  if (Instance()->param_sets.empty())
    ROS_ERROR("Couldn't load any parameters!");
}

void ParameterManager::updateParameterSet(const ParameterSet& params)
{
  Instance()->param_sets[params.getName()] = params;
  ROS_INFO("Updated parameter set '%s'.", params.getName().c_str());
}

void ParameterManager::updateParameterSet(const msgs::ParameterSet& params)
{
  Instance()->param_sets[params.name.data].fromMsg(params);
  ROS_INFO("Updated parameter set '%s'.", params.name.data.c_str());
}

bool ParameterManager::getParameterSet(const std::string& name, ParameterSet& params)
{
  std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets.find(name);
  if (itr == Instance()->param_sets.end())
    return false;

  params = itr->second;
  return true;
}

bool ParameterManager::getParameterSet(const std::string& name, msgs::ParameterSet& params)
{
  std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets.find(name);
  if (itr == Instance()->param_sets.end())
    return false;

  itr->second.toMsg(params);
  return true;
}

void ParameterManager::getAllParameterSets(std::vector<ParameterSet>& param_sets)
{
  param_sets.clear();
  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets.begin(); itr != Instance()->param_sets.end(); itr++)
    param_sets.push_back(itr->second);
}

void ParameterManager::getAllParameterSets(std::vector<msgs::ParameterSet>& param_sets)
{
  param_sets.clear();
  msgs::ParameterSet params;
  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets.begin(); itr != Instance()->param_sets.end(); itr++)
  {
    itr->second.toMsg(params);
    param_sets.push_back(params);
  }
}

void ParameterManager::removeParameterSet(const std::string& name)
{
  Instance()->param_sets.erase(name);

  if (Instance()->active_parameter_set->getName() == name)
    Instance()->active_parameter_set.reset(new ParameterSet());
}

bool ParameterManager::hasParameterSet(const std::string& name)
{
  return Instance()->param_sets.find(name) != Instance()->param_sets.end();
}

void ParameterManager::getParameterSetNames(std::vector<std::string>& names)
{
  names.clear();

  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets.begin(); itr != Instance()->param_sets.end(); itr++)
    names.push_back(itr->first);
}

void ParameterManager::getParameterSetNames(std::vector<std_msgs::String>& names)
{
  names.clear();

  std_msgs::String name;
  for (std::map<std::string, ParameterSet>::const_iterator itr = Instance()->param_sets.begin(); itr != Instance()->param_sets.end(); itr++)
  {
    name.data = itr->first;
    names.push_back(name);
  }
}

bool ParameterManager::setActive(const std::string& name)
{
  if (getParameterSet(name, *Instance()->active_parameter_set))
  {
    ROS_INFO("[ParameterManager] Set '%s' as active parameter set.", name.c_str());
  }
  else
  {
    ROS_ERROR("[ParameterManager] Can't set '%s' as active parameter set!", name.c_str());
    return false;
  }
  return true;
}

const ParameterSet& ParameterManager::getActive()
{
  return *(Instance()->active_parameter_set);
}

// --- Subscriber calls ---

void ParameterManager::updateParameterSet(const msgs::ParameterSetConstPtr& params)
{
  updateParameterSet(*params);
}

// --- Service calls ---

bool ParameterManager::setParameterSetService(msgs::SetParameterSetService::Request& req, msgs::SetParameterSetService::Response& resp)
{
  updateParameterSet(req.params);
  return true;
}

bool ParameterManager::getParameterSetService(msgs::GetParameterSetService::Request& req, msgs::GetParameterSetService::Response& resp)
{
  getParameterSet(req.name.data, resp.params);
  return true;
}

bool ParameterManager::getAllParameterSetsService(msgs::GetAllParameterSetsService::Request& req, msgs::GetAllParameterSetsService::Response& resp)
{
  getAllParameterSets(resp.param_sets);
  return true;
}

bool ParameterManager::getParameterSetNamesService(msgs::GetParameterSetNamesService::Request& req, msgs::GetParameterSetNamesService::Response& resp)
{
  getParameterSetNames(resp.names);
  return true;
}

//--- action server calls ---

void ParameterManager::setParameterSetAction(SimpleActionServer<msgs::SetParameterSetAction>::Ptr& as)
{
  const msgs::SetParameterSetGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::SetParameterSetResult result;
  updateParameterSet(goal->params);

  as->finish(result);
}

void ParameterManager::getParameterSetAction(SimpleActionServer<msgs::GetParameterSetAction>::Ptr& as)
{
  const msgs::GetParameterSetGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GetParameterSetResult result;
  if (!getParameterSet(goal->name.data, result.params))
    result.status = ErrorStatusError(msgs::ErrorStatus::ERR_UNKNOWN, "ParameterManager", "getParameterSetAction: Couldn't get params named '" + goal->name.data + "'");

  as->finish(result);
}

void ParameterManager::getAllParameterSetsAction(SimpleActionServer<msgs::GetAllParameterSetsAction>::Ptr& as)
{
  const msgs::GetAllParameterSetsGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GetAllParameterSetsResult result;
  getAllParameterSets(result.param_sets);

  as->finish(result);
}

void ParameterManager::getParameterSetNamesAction(SimpleActionServer<msgs::GetParameterSetNamesAction>::Ptr& as)
{
  const msgs::GetParameterSetNamesGoalConstPtr& goal(as->acceptNewGoal());

  // check if new goal was preempted in the meantime
  if (as->isPreemptRequested())
  {
    as->setPreempted();
    return;
  }

  msgs::GetParameterSetNamesResult result;
  getParameterSetNames(result.names);

  as->finish(result);
}
}
