#pragma once

#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/back/state_machine.hpp>

#include <ros/ros.h>
#include "bag_recorder/outgoing_message.h"

namespace msm = boost::msm;

namespace queue_state
{
struct queue_idle : msm::front::state<>
{
  template <class event_t, class fsm_t>
  void on_entry(event_t const&, fsm_t&)
  {
    ROS_DEBUG("entry: idle");
  }

  template <class event_t, class fsm_t>
  void on_exit(event_t const&, fsm_t&)
  {
    ROS_DEBUG("exit: idle");
  }
};
struct queue_filling : msm::front::state<>
{
  template <class event_t, class fsm_t>
  void on_entry(event_t const&, fsm_t&)
  {
    ROS_DEBUG("entry: filling");
  }

  template <class event_t, class fsm_t>
  void on_exit(event_t const&, fsm_t&)
  {
    ROS_DEBUG("exit: filling");
  }
};
struct queue_filled : msm::front::state<>
{
  template <class event_t, class fsm_t>
  void on_entry(event_t const&, fsm_t&)
  {
    ROS_DEBUG("entry: filled");
  }

  template <class event_t, class fsm_t>
  void on_exit(event_t const&, fsm_t&)
  {
    ROS_DEBUG("exit: filled");
  }
};
struct queue_recording : msm::front::state<>
{
  template <class event_t, class fsm_t>
  void on_entry(event_t const&, fsm_t&)
  {
    ROS_DEBUG("entry: recording");
  }

  template <class event_t, class fsm_t>
  void on_exit(event_t const&, fsm_t&)
  {
    ROS_DEBUG("exit: recording");
  }
};
struct queue_closing : msm::front::state<>
{
  template <class event_t, class fsm_t>
  void on_entry(event_t const&, fsm_t&)
  {
    ROS_DEBUG("entry: closing");
  }

  template <class event_t, class fsm_t>
  void on_exit(event_t const&, fsm_t&)
  {
    ROS_DEBUG("exit: closing");
  }
};

struct open_queue
{
  std::shared_ptr<bag_recorder::message_queue_t> queue;
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
  bag_recorder::OutgoingMessage* out;
};

struct queue_msm_ : msm::front::state_machine_def<queue_msm_>
{
  void open(open_queue const& ev)
  {
    message_queue_    = ev.queue;
    queue_start_time_ = ros::Time::now();
  }
  void start(start_recording const&)
  {
    record_start_time_ = ros::Time::now();
  }
  bool push_filling(push_message const& ev)
  {
    message_queue_->push(*(ev.out));
    ros::Duration queue_duration = ros::Time::now() - queue_start_time_;
    return (queue_duration > max_queue_duration_);
  }
  bool push_filled(push_message const& ev)
  {
    message_queue_->push(*(ev.out));
    message_queue_->pop();
    return false;
  }
  bool push_recording(push_message const& ev)
  {
    message_queue_->push(*(ev.out));
    ros::Duration record_duration = ros::Time::now() - record_start_time_;
    return (record_duration > max_record_duration_);
  }
  bool push_closing(push_message const&)
  {
    queue_start_time_ = ros::Time::now();
    return (message_queue_->empty());
  }
  struct transition_table : boost::mpl::vector
                            //    source      | event     | target
                            <a_row<queue_idle, open_queue, queue_filling, &queue_msm_::open>,
                             a_row<queue_filling, start_recording, queue_recording, &queue_msm_::start>,
                             g_row<queue_filling, push_message, queue_filled, &queue_msm_::push_filling>,
                             a_row<queue_filled, start_recording, queue_recording, &queue_msm_::start>,
                             g_row<queue_filled, push_message, queue_recording, &queue_msm_::push_filled>,
                             _row<queue_recording, stop_recording, queue_closing>,
                             g_row<queue_recording, push_message, queue_closing, &queue_msm_::push_recording>,
                             g_row<queue_closing, push_message, queue_filling, &queue_msm_::push_closing> >
  {
  };

  using initial_state = queue_idle;

  ros::Time     queue_start_time_;
  ros::Time     record_start_time_;
  ros::Duration max_queue_duration_;
  ros::Duration max_record_duration_;

  std::shared_ptr<bag_recorder::message_queue_t> message_queue_;
  template <class FSM, class Event>
  void no_transition(Event const& e, FSM&, int state)
  {
    ROS_DEBUG("no transition from state %d on event %s", state, typeid(e).name());
  }

public:
  queue_msm_(double max_queue_duration = 3.0, double max_record_duration = 10.0)
    : max_queue_duration_(max_queue_duration), max_record_duration_(max_record_duration)
  {
  }
};

using queue_msm = boost::msm::back::state_machine<queue_msm_>;
};  // namespace queue_state
