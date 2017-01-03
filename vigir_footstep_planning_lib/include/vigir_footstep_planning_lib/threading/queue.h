//=================================================================================================
// Copyright (c) 2017, Alexander Stumpf, TU Darmstadt
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

#ifndef VIGIR_FOOTSTEP_PLANNING_THREADING_QUEUE_H__
#define VIGIR_FOOTSTEP_PLANNING_THREADING_QUEUE_H__

#include <ros/ros.h>

#include <queue>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>



namespace vigir_footstep_planning
{
namespace threading
{
template <class T>
class Queue
{
public:
  Queue()
    : job_counter(0)
  {
  }

  virtual ~Queue()
  {
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex);
    queued_jobs = typename std::queue<boost::shared_ptr<T> >();
    job_counter = 0;
    jobs_finished_condition.notify_all();
  }

  void queueJob(boost::shared_ptr<T>& job)
  {
    { // scope needed so mutex gets unlocked when other threads are notified
      boost::mutex::scoped_lock lock(queued_jobs_mutex);
      queued_jobs.push(job);
    }
    queued_jobs_condition.notify_one();
  }

  void queueJobs(std::list<boost::shared_ptr<T> >& jobs)
  {
    { // scope needed so mutex gets unlocked when other threads are notified
      boost::mutex::scoped_lock lock(queued_jobs_mutex);
      for (typename std::list<boost::shared_ptr<T> >::iterator itr = jobs.begin(); itr != jobs.end(); itr++)
        queued_jobs.push(*itr);
    }
    queued_jobs_condition.notify_all();
  }

  boost::shared_ptr<T>& waitAndDequeueJob()
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex);

    if (queued_jobs.empty())
    {
      ROS_DEBUG("[Queue] Waiting for Job getting queued...");
      queued_jobs_condition.wait(lock);
    }

    ROS_DEBUG("[Queue] Pop Job.");

    boost::shared_ptr<T>& front = queued_jobs.front();
    queued_jobs.pop();
    job_counter++;

    return front;
  }

  void waitAndDequeueJobs(std::vector<boost::shared_ptr<T> >& jobs, unsigned int n)
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex);

    if (queued_jobs.empty())
    {
      ROS_DEBUG("[Queue] Waiting for Job getting queued...");
      queued_jobs_condition.wait(lock);
    }

    ROS_DEBUG("[Queue] Pop %u Job(s).", n);
    n = std::min(n, static_cast<unsigned int>(queued_jobs.size()));
    jobs.resize(n);

    for (unsigned int i = 0; i < n; i++)
    {
      jobs[i] = queued_jobs.front();
      queued_jobs.pop();
      job_counter++;
    }
  }

  void justFinishedJobs(unsigned int n)
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex);

    if (job_counter > n)
      job_counter -= n;
    else
      job_counter = 0;

    if (queued_jobs.empty() && job_counter == 0)
    {
      ROS_DEBUG("[Queue] Notify for finished job.");
      jobs_finished_condition.notify_all();
    }
  }

  bool hasOpenJobs() const
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex);
    return !queued_jobs.empty() || job_counter != 0;
  }

  void waitUntilJobsProcessed() const
  {
    boost::mutex::scoped_lock lock(queued_jobs_mutex);

    if (!queued_jobs.empty())
    {
      ROS_DEBUG("[Queue] Waiting for Jobs getting finished...");
      jobs_finished_condition.wait(lock);
      ROS_DEBUG("[Queue] Waiting for Jobs getting finished...Done!");
    }
  }

protected:
  std::queue<boost::shared_ptr<T> > queued_jobs;
  unsigned int job_counter;

  mutable boost::mutex queued_jobs_mutex;

  mutable boost::condition_variable queued_jobs_condition;
  mutable boost::condition_variable jobs_finished_condition;
};
}
}

#endif
