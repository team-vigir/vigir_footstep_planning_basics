//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_THREADING_MANAGER_H__
#define VIGIR_FOOTSTEP_PLANNING_THREADING_MANAGER_H__

#include <ros/ros.h>

#include <vigir_footstep_planning_lib/threading/queue.h>
#include <vigir_footstep_planning_lib/threading/worker.h>



namespace vigir_footstep_planning
{
namespace threading
{
template <class T>
class ThreadingManager
{
public:
  ThreadingManager(int threads = -1, unsigned int jobs_per_thread = 10, bool auto_start = true)
  {
    ROS_INFO("[Manager] Spawning %i workers.", threads);
    for (int n = 0; n < threads; n++)
      workers.push_back(boost::shared_ptr<Worker<T> >(new Worker<T>(queue, jobs_per_thread, auto_start)));
  }

  virtual ~ThreadingManager()
  {
    ROS_INFO("[Manager] Destruct");
    stopJobs();
  }

  void addJob(boost::shared_ptr<T>& job) { queue.queueJob(job); }
  void addJobs(std::list<boost::shared_ptr<T> >& jobs) { queue.queueJobs(jobs); }

  void stopJobs()
  {
    ROS_INFO("[Manager] Wait for thread termination...");
    for (typename std::list<boost::shared_ptr<Worker<T> > >::iterator itr = workers.begin(); itr != workers.end(); itr++)
      (*itr)->stop();
    deleteJobs();
    ROS_INFO("[Manager] Wait for thread termination...Done!");
  }

  void interruptJobs()
  {
    ROS_INFO("[Manager] Interrupt jobs...");
    for (typename std::list<boost::shared_ptr<Worker<T> > >::iterator itr = workers.begin(); itr != workers.end(); itr++)
      (*itr)->interruptJobs();
    deleteJobs();
    ROS_INFO("[Manager] Interrupt jobs...Done!");
  }

  void deleteJobs() { queue.clear(); }

  bool hasJobsFinished() { return !queue.hasOpenJobs(); }
  void waitUntilJobsFinished()
  {
    try
    {
      queue.waitUntilJobsProcessed();
    }
    catch(boost::thread_interrupted& interrupt)
    {
      ROS_INFO("[Manager] Catched boost::interruption");
      interruptJobs();
      throw(interrupt);
    }
  }

  // typedefs
  typedef boost::shared_ptr<ThreadingManager> Ptr;
  typedef boost::shared_ptr<const ThreadingManager> ConstPtr;

protected:
  Queue<T> queue;
  std::list<boost::shared_ptr<Worker<T> > > workers;
};
}
}

#endif
