#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/publisher.h>
#include <boost/thread/mutex.hpp>


class GenericTopicDiagnostic
{
public:
  GenericTopicDiagnostic(std::string& topic_name, diagnostic_updater::Updater& diagnostic_updater)
  {
    ros::NodeHandle nh;
    //ros::param::get("~min_freq", min_freq_);
    //ros::param::get("~max_freq", max_freq_);
    double hz, hzerror;
    ros::param::get("~hz", hz);
    ros::param::get("~hzerror", hzerror);
    min_freq_ = hz-hzerror;
    max_freq_ = hz+hzerror;

    diagnostic_updater::FrequencyStatusParam freq_param(&min_freq_, &max_freq_, 0.1, 5); //min_freq, max_freq, tolerance (default: 0.1), window_size (default: 5)
    diagnostic_updater::TimeStampStatusParam stamp_param(-1, 1); //min_acceptable (default: -1), max_acceptable (default: 5)

    topic_diagnostic_task_.reset(new diagnostic_updater::TopicDiagnostic(topic_name, diagnostic_updater, freq_param, stamp_param));

    generic_sub_ = nh.subscribe<topic_tools::ShapeShifter>(topic_name, 1, &GenericTopicDiagnostic::topicCallback, this);
  }

  void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg)
  {
    topic_diagnostic_task_->tick(ros::Time::now());
  }

  double min_freq_, max_freq_;
  ros::Subscriber generic_sub_;
  boost::shared_ptr< diagnostic_updater::TopicDiagnostic > topic_diagnostic_task_;
};


class TopicStatusMonitor
{
public:
  TopicStatusMonitor()
  {
    ros::NodeHandle nh;
    ros::param::get("~topics", topics_);
    ros::param::get("~diagnostic_name", hardware_id_);
    diagnostic_updater_.setHardwareID(hardware_id_);
    
    for(size_t i=0; i<topics_.size(); i++)
    {
      tasks_.push_back(new GenericTopicDiagnostic (topics_[i], diagnostic_updater_));
    }

    diagnostic_timer_ = nh.createTimer(ros::Duration(1.0), &TopicStatusMonitor::updateDiagnostics, this);
    diagnostic_timer_.start();
  }

  ~TopicStatusMonitor()
  {}
  

  void updateDiagnostics(const ros::TimerEvent& event)
  {
    diagnostic_updater_.update();
  }

private:
  std::string hardware_id_;
  std::vector<std::string> topics_;
  diagnostic_updater::Updater diagnostic_updater_;

  std::vector< GenericTopicDiagnostic* > tasks_;
  ros::Timer diagnostic_timer_;
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "topic_status_monitor");
  TopicStatusMonitor tsm;

  ros::spin();

  return 0;
}
