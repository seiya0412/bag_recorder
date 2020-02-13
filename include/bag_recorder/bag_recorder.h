/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/
#pragma once

// Main ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <topic_tools/shape_shifter.h>

// rosbag tools
#include "rosbag/bag.h"
#include "rosbag/stream.h"
#include "rosbag/macros.h"

// Thread management
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

// std tools
#include <queue>
#include <string>
#include <vector>

#include "bag_recorder/outgoing_message.h"


namespace bag_recorder
{
class QueueTransaction;
class QueueState;
class QueueIdle;
class QueueFilling;
class QueueFilled;
class QueueRecording;
class QueueClosing;
class OutgoingMessage
{
public:
  OutgoingMessage(std::string const &                 _topic,
                  topic_tools::ShapeShifter::ConstPtr _msg,
                  boost::shared_ptr<ros::M_string>    _connection_header,
                  ros::Time                           _time);

  std::string                         topic;
  topic_tools::ShapeShifter::ConstPtr msg;
  boost::shared_ptr<ros::M_string>    connection_header;
  ros::Time                           time;
};  // OutgoingMessage
using message_queue_t = std::queue<OutgoingMessage>;

class QueueState
{
public:
  virtual bool open(QueueTransaction *)
  {
    return false;
  };
  virtual bool close(QueueTransaction *)
  {
    return false;
  };
  virtual bool start(QueueTransaction *)
  {
    return false;
  };
  virtual bool stop(QueueTransaction *)
  {
    return false;
  };
  virtual bool push(QueueTransaction *, OutgoingMessage)
  {
    return false;
  };

protected:
  void changeState(QueueTransaction *q, QueueState *s)
  {
    q->changeState(s);
  }

  void setQueueStartTime(QueueTransaction *q, const ros::Time &t)
  {
    q->setQueueStartTime(t);
  }

  const ros::Time getQueueStartTime(QueueTransaction *q)
  {
    return q->getQueueStartTime();
  }

  void setRecordStartTime(QueueTransaction *q, const ros::Time &t)
  {
    q->setRecordStartTime(t);
  }

  const ros::Time getRecordStartTime(QueueTransaction *q)
  {
    return q->getRecordStartTime();
  }

  const ros::Duration getMaxQueueDuration(QueueTransaction *q)
  {
    return q->getMaxQueueDuration();
  }
  const ros::Duration getMaxRecordDuration(QueueTransaction *q)
  {
    return q->getMaxRecordDuration();
  }
  std::shared_ptr<message_queue_t> grabMessageQueue(QueueTransaction *q)
  {
    return q->grabMessageQueue();
  }

  const bool empty(QueueTransaction *q)
  {
    return q->empty();
  }
};

class QueueTransaction
{
public:
  QueueTransaction()
    : queue_start_time_(0.0), record_start_time_(0.0), max_queue_duration_(0.0), max_record_duration_(0.0)
  {
    state_ = bag_recorder::QueueIdle::instance();
  }

  bool open()
  {
    return state_->open(this);
  }

  bool close()
  {
    return state_->close(this);
  }

  bool start()
  {
    return state_->start(this);
  }

  bool stop()
  {
    return state_->stop(this);
  }

  bool push(OutgoingMessage msg)
  {
    return state_->push(this, msg);
  }

  void attachMessageQueue(std::shared_ptr<message_queue_t> message_queue)
  {
    message_queue_ = message_queue;
  }

private:
  friend class QueueState;
  void changeState(QueueState *s)
  {
    state_ = s;
  }

  void setQueueStartTime(const ros::Time &t)
  {
    queue_start_time_ = t;
  }

  const ros::Time getQueueStartTime()
  {
    return queue_start_time_;
  }

  void setRecordStartTime(const ros::Time &t)
  {
    record_start_time_ = t;
  }

  const ros::Time getRecordStartTime()
  {
    return record_start_time_;
  }

  const ros::Duration getMaxQueueDuration()
  {
    return max_queue_duration_;
  }

  const ros::Duration getMaxRecordDuration()
  {
    return max_record_duration_;
  }

  std::shared_ptr<message_queue_t> grabMessageQueue()
  {
    return message_queue_;
  }

  bool empty()
  {
    return message_queue_ == nullptr;
  }

private:
  QueueState *                     state_;
  ros::Time                        queue_start_time_;
  ros::Time                        record_start_time_;
  ros::Duration                    max_queue_duration_;
  ros::Duration                    max_record_duration_;
  std::shared_ptr<message_queue_t> message_queue_;
};

class QueueIdle : public QueueState
{
public:
  static QueueState *instance()
  {
    if(instance_ == nullptr)
    {
      instance_ = new QueueIdle;
    }

    return instance_;
  }

  bool open(QueueTransaction *q)
  {
    if(empty(q))
      return false;
    setQueueStartTime(q, ros::Time::now());
    changeState(q, QueueFilling::instance());
    return true;
  }

protected:
  QueueIdle()
  {
    instance_ = nullptr;
  }

private:
  static QueueState *instance_;
};

class QueueFilling : public QueueState
{
public:
  static QueueState *instance()
  {
    if(instance_ == nullptr)
    {
      instance_ = new QueueFilling;
    }

    return instance_;
  }

  bool start(QueueTransaction *q)
  {
    if(empty(q))
      return false;
    setRecordStartTime(q, ros::Time::now());
    changeState(q, QueueRecording::instance());
    return true;
  }

  bool push(QueueTransaction *q, OutgoingMessage msg)
  {
    if(empty(q))
      return false;
    std::shared_ptr<message_queue_t> message_queue  = grabMessageQueue(q);
    ros::Duration                    queue_duration = ros::Time::now() - getQueueStartTime(q);
    if(queue_duration > getMaxQueueDuration(q))
    {
      changeState(q, QueueFilled::instance());
    }
    else
    {
      message_queue->push(msg);
    }
    return true;
  }

protected:
  QueueFilling()
  {
    instance_ = nullptr;
  }

private:
  static QueueState *instance_;
};

class QueueFilled : public QueueState
{
public:
  static QueueState *instance()
  {
    if(instance_ == nullptr)
    {
      instance_ = new QueueFilled;
    }

    return instance_;
  }

  bool start(QueueTransaction *q)
  {
    if(empty(q))
      return false;
    setRecordStartTime(q, ros::Time::now());
    changeState(q, QueueRecording::instance());
    return true;
  }

  bool push(QueueTransaction *q, OutgoingMessage msg)
  {
    if(empty(q))
      return false;
    std::shared_ptr<message_queue_t> message_queue = grabMessageQueue(q);
    message_queue->push(msg);
    message_queue->pop();
    return true;
  }

protected:
  QueueFilled()
  {
    instance_ = nullptr;
  }

private:
  static QueueState *instance_;
};

class QueueRecording : public QueueState
{
public:
  static QueueState *instance()
  {
    if(instance_ == nullptr)
    {
      instance_ = new QueueRecording;
    }

    return instance_;
  }

  bool stop(QueueTransaction *q)
  {
    if(empty(q))
      return false;
    changeState(q, QueueClosing::instance());
    return true;
  }

  bool push(QueueTransaction *q, OutgoingMessage msg)
  {
    if(empty(q))
      return false;
    std::shared_ptr<message_queue_t> message_queue = grabMessageQueue(q);
    message_queue->push(msg);
    setQueueStartTime(q, ros::Time::now());
    return true;
  }

protected:
  QueueRecording()
  {
    instance_ = nullptr;
  }

private:
  static QueueState *instance_;
};

class QueueClosing : public QueueState
{
public:
  static QueueState *instance()
  {
    if(instance_ == nullptr)
    {
      instance_ = new QueueClosing;
    }

    return instance_;
  }

  bool push(QueueTransaction *q, OutgoingMessage msg)
  {
    if(empty(q))
      return false;
    std::shared_ptr<message_queue_t> message_queue = grabMessageQueue(q);
    if(message_queue->empty())
    {
      changeState(q, QueueFilling::instance());
    }
    return true;
  }

protected:
  QueueClosing()
  {
    instance_ = nullptr;
  }

private:
  static QueueState *instance_;
};

enum class RecorderMode
{
  DEFAULT = 0,
  DRIVING_RECORDER,
};

class BagRecorder
{
public:
  // initializes the BagRecorder
  BagRecorder(std::string data_folder, bool append_date = true, RecorderMode recorder_mode = RecorderMode::DEFAULT);
  ~BagRecorder();

  // flow control function - sets flags, starts threads, starts/kills subscribers
  bool start_queueing(std::vector<std::string> topics,
                      bool record_all_topics = false);  //-- initializes subscribers starts record thread, initializes
                                                        // all record variables, generates bag_ name
  std::string start_recording(std::string bag__name);   //-- initializes subscribers starts record thread, initializes
                                                        // all record variables, generates bag_ name
  void stop_recording();  //-- kills subscribers, sets flag for write_thread to stop after queue is cleared
  void stop_queueing();

  void immediate_stop_recording();  //-- kills subscribers, sets flag for write_thread to stop recording immediately

  // status check functions
  bool is_active();                          //-- if there is a bag_ being recorded to
  bool can_log();                            //-- if the BagRecorder can atually log to the file
  bool is_subscribed_to(std::string topic);  //-- if the BagRecorder is currently subscribed to this topic

  std::string get_bagname();  //-- gets the bag name currently being recoreded to
  QueueState  get_queue_state();

private:
  // generates a subcriber
  void                               subscribe_all();
  boost::shared_ptr<ros::Subscriber> generate_subscriber(std::string const &topic);
  void subscriber_callback(const ros::MessageEvent<topic_tools::ShapeShifter const> &msg_event,
                           std::string const &                                       topic,
                           boost::shared_ptr<ros::Subscriber>                        subscriber,
                           boost::shared_ptr<int>                                    count);
  void unsubscribe_all();

  // write thread
  void queue_processor();  //-- starts bag_, writes to queue until end conditin reached, closes bag_

  // helper functions to check for errors
  void run_scheduled_checks();  //-- sees if check_disk is scheduled, run if so and schedule next check
  void check_disk();            //-- checks disk to see if there is space to write or whatnot

  // helper function to generate bag__name
  static std::string get_time_str();  //-- turns  ros time into string

private:
  // my data
  std::string        data_folder_;
  bool               append_date_;
  RecorderMode       recorder_mode_;
  bool               recording_all_topics_;
  unsigned long long min_recording_space_     = 1024 * 1024 * 1024;
  std::string        min_recording_space_str_ = "1G";

  // bag_ data
  rosbag::Bag bag_;
  std::string bag_filename_ = "";     // initialized in cas get_bagname is called
  bool        bag_active_   = false;  // is by default not active

  // stop signals
  bool         clear_queue_signal_;
  bool         stop_signal_;
  boost::mutex start_stop_mutex_;

  // subscriber data
  std::vector<boost::shared_ptr<ros::Subscriber>> subscribers_;
  std::set<std::string>                           subscribed_topics_;
  ros::WallTime                                   subscribe_all_next_;
  boost::mutex subscribers_mutex_;  //-- mutex for the subscribers queue and recording_all_topics

  // thread data control
  boost::condition_variable_any    queue_condition_;
  boost::mutex                     queue_mutex_;
  std::shared_ptr<message_queue_t> message_queue_;  //!< queue for storing
  QueueTransaction                 queue_transaction_;

  boost::thread record_thread_;

  // disk checking variables
  bool          checks_failed_ = false;  // by default no checks failed
  ros::WallTime check_disk_next_;
  ros::WallTime warn_next_;

  // probably shouldn't actually be set here :P
  // but I can't think of a better way to reveal these so...
  double subscribe_all_interval_ = 5.0;
  double check_disk_interval_    = 10.0;
};  // BagRecorder

}  // namespace bag_recorder
