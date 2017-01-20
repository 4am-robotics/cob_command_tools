#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/publisher.h>
#include <boost/thread/mutex.hpp>



class TopicStatusMonitor
{
public:
  TopicStatusMonitor()
  {
    ros::NodeHandle nh;
  
    std::string hardware_id;
    ros::param::get("~hardware_id", hardware_id);
    std::string topic_name;
    ros::param::get("~topic_name", topic_name);
    double min_freq;
    ros::param::get("~min_freq", min_freq_);

    ros::param::get("~max_freq", max_freq_);

    diagnostic_updater_.setHardwareID(hardware_id);

    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq_, &max_freq_, 0.1, 5); //min_freq, max_freq, tolerance (default: 0.1), window_size (default: 5)
    diagnostic_updater::TimeStampStatusParam stamp_param(-1, 1); //min_acceptable (default: -1), max_acceptable (default: 5)
    topic_diagnostic_task_.reset(new diagnostic_updater::TopicDiagnostic(topic_name, diagnostic_updater_, freq_param, stamp_param));

    last_message_received_ = ros::Time(0);
    timeout_ = ros::Duration(5.0);  //default stale timeout
    generic_sub_ = nh.subscribe<topic_tools::ShapeShifter>(topic_name, 1, &TopicStatusMonitor::topicCallback, this);
    diagnostic_timer_ = nh.createTimer(ros::Duration(1.0), &TopicStatusMonitor::updateDiagnostics, this);

    diagnostic_timer_.start();
  }

  ~TopicStatusMonitor()
  {}
  

  void updateDiagnostics(const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(ros::Time::now() - last_message_received_ < timeout_)
      diagnostic_updater_.update();
  }

  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    last_message_received_ = ros::Time::now();
    topic_diagnostic_task_->tick(last_message_received_);
  }

private:
  diagnostic_updater::Updater diagnostic_updater_;
  boost::shared_ptr< diagnostic_updater::TopicDiagnostic > topic_diagnostic_task_;

  double min_freq_, max_freq_;

  ros::Time last_message_received_;
  ros::Duration timeout_;
  boost::mutex mutex_;
  ros::Subscriber generic_sub_;
  ros::Timer diagnostic_timer_;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_status_monitor");
  TopicStatusMonitor tsm;

  ros::spin();

  return 0;
}
