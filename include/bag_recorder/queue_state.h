#pragma once

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>

#include <ros/ros.h>

namespace msm = boost::msm;

namespace queue_state
{
struct queue_idle : msm::front::state<>
{
};
struct queue_filling : msm::front::state<>
{
};
struct queue_filled : msm::front::state<>
{
};
struct queue_recording : msm::front::state<>
{
};
struct queue_closing : msm::front::state<>
{
};

struct open_queue
{
};
struct close_queue
{
};
struct start_recording
{
};
struct stop_recording
{
};
struct push_message
{
};

struct queue_msm_ : msm::front::state_machine_def<queue_msm_>
{
  bool push_filling(push_message const& ev)
  {
    return true;
  }
  bool push_filled(push_message const& ev)
  {
    return true;
  }
  bool push_recording(push_message const& ev)
  {
    return true;
  }
  bool push_closing(push_message const& ev)
  {
    return true;
  }
  struct transition_table : boost::mpl::vector
                            //    source      | event     | target
                            <_row<queue_idle, open_queue, queue_filling>,
                             _row<queue_filling, start_recording, queue_recording>,
                             g_row<queue_filling, push_message, queue_filled, &queue_msm_::push_filling>,
                             _row<queue_filled, start_recording, queue_recording>,
                             g_row<queue_filling, push_message, queue_recording, &queue_msm_::push_filled>,
                             _row<queue_recording, stop_recording, queue_closing>,
                             g_row<queue_recording, push_message, queue_closing, &queue_msm_::push_recording>,
                             g_row<queue_closing, push_message, queue_filling, &queue_msm_::push_closing> >
  {
  };

  using initial_state = queue_idle;
};

using queue_msm = boost::msm::back::state_machine<queue_msm_>;
};  // namespace queue_state