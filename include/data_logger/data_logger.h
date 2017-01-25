/* Author: Mikhail Medvedev */

#ifndef DATA_LOGGER_H_
#define DATA_LOGGER_H_

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <data_logger/status_publisher.h>
#include "data_logger/start.h"

namespace data_logger
{

class DataLogger
{
public:
  DataLogger();
  virtual ~DataLogger();

private:
  ros::NodeHandle nh_;
  ros::ServiceServer srv_start_;
  ros::ServiceServer srv_stop_;
  bool start(data_logger::start::Request &req, data_logger::start::Response &res);
  bool stop(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
 
  int pipefd[2];
  pid_t bag_process_pid_;
  std::string rosbag_record_args_;
  std::string bag_path_;

  // Publish data logger status.
  StatusPublisher status_pub_;
};

} // namespace data_logger

#endif /* DATA_LOGGER_H_ */

