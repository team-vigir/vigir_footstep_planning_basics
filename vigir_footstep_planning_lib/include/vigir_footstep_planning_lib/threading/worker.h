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

#ifndef VIGIR_FOOTSTEP_PLANNING_THREADING_WORKER_H__
#define VIGIR_FOOTSTEP_PLANNING_THREADING_WORKER_H__

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <vigir_footstep_planning_lib/threading/queue.h>



namespace vigir_footstep_planning
{
namespace threading
{
template <class T>
class Worker
{
public:
  Worker(Queue<T>& queue, unsigned int jobs_per_thread = 10, bool auto_start = true)
    : queue(queue)
    , jobs_per_thread(jobs_per_thread)
    , exit(false)
  {
    if (auto_start)
      start();
  }

  virtual ~Worker()
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() <<")] Destruct");
    stop();
  }

  void start()
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() <<")] Start request");
    exit = false;
    thread = boost::thread(&Worker::run, this, jobs_per_thread);
  }

  void stop()
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() <<")] Stop request");

    // soft stop
    interruptJobs();
    exit = true;

    // hard stop
    if (thread.joinable())
    {
      thread.interrupt();
      thread.join();
    }
  }

  void interruptJobs()
  {
    boost::mutex::scoped_lock lock(run_jobs_mutex);
    run_jobs = false;
  }

protected:
  void run(unsigned int n)
  {
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() <<")] Started with " << n << " jobs per thread.");
    std::vector<boost::shared_ptr<T> > jobs;
    while (!exit)
    {
      queue.waitAndDequeueJobs(jobs, n);
      {
        boost::mutex::scoped_lock lock(run_jobs_mutex);
        run_jobs = true;
      }

      ROS_DEBUG_STREAM("[Worker (" << boost::this_thread::get_id() <<")] Deqeued " << jobs.size() << " jobs.");
      for (size_t i = 0; i < jobs.size() && run_jobs; i++)
      {
        boost::this_thread::interruption_point();
        jobs[i]->run();
      }

      if (run_jobs)
        queue.justFinishedJobs(jobs.size());
    }
    ROS_INFO_STREAM("[Worker (" << boost::this_thread::get_id() <<")] Stopped!");
  }

  boost::thread thread;
  mutable boost::mutex run_jobs_mutex;

  Queue<T>& queue;
  unsigned int jobs_per_thread;
  bool exit;
  bool run_jobs;
};
}
}

#endif
