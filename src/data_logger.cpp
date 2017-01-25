/* Author: Mikhail Medvedev */

#include <data_logger/data_logger.h>

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include <sys/wait.h>
#include <boost/filesystem.hpp>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "data_logger");
  data_logger::DataLogger dl;
  return 0;
}

namespace data_logger
{
DataLogger::DataLogger() :
        nh_("~"),
        srv_start_(nh_.advertiseService("start", &DataLogger::start, this)),
        srv_stop_(nh_.advertiseService("stop", &DataLogger::stop, this)),
        bag_process_pid_(0)

{
  nh_.param("rosbag_record_args", rosbag_record_args_, std::string(""));
  nh_.param("bag_path", bag_path_, std::string("~/bag"));
  status_pub_.pubStopped();
  pipe(pipefd);
  // If start_recording is set to true, start bag record as soon as the node is started
  {
    bool start_recording = false;
    nh_.param("start_recording", start_recording, false);
    /*
       need to handle this default case, defaults are set so maybe check
       in service callback for empty req and handle appropriately
    */
    if (start_recording) 
    {
      ROS_INFO("start_recording is true, calling 'start'");
      data_logger::start::Request req;
      data_logger::start::Response res;
      //std_srvs::EmptyRequest empty_req;
      //std_srvs::EmptyResponse empty_res;
      start(req, res);
    }
  }

  ros::spin();
}

DataLogger::~DataLogger()
{
  killpg(bag_process_pid_, SIGINT);
  status_pub_.pubStopped();
}

bool DataLogger::start(data_logger::start::Request &req, 
                       data_logger::start::Response &res)
{
  // Start child 'bag record'

  if (bag_process_pid_ != 0) // Allow only one recording process at a time
  {
    return false;
  }

  status_pub_.pubStarted();
  bag_process_pid_ = fork();
  switch (bag_process_pid_)
  {
    default: // In Parent
    {
      ROS_INFO_STREAM("Started new recording process, pid: "<<bag_process_pid_);
      //poll for topics
    /*  char buffer[256];
      close(pipefd[1]);
      for(;;)
      {
          while(read(pipefd[0], buffer, sizeof(buffer)) != 0)
          {
          
              ROS_INFO_STREAM("STUFF: " << buffer << "\n");
          }
          ROS_INFO_STREAM("NOTHING");
      }*/      
      break;
    }
    case 0: // In child
    {
     /*
     close(pipefd[0]);
     dup2(pipefd[1], 1);
    // dup2(pipefd[1], 2);
     close(pipefd[1]); */
      // Check if directory exists, create
      if (!boost::filesystem::exists(bag_path_))
      {
        boost::filesystem::create_directories(bag_path_);
      }
      if (chdir(req.bag_path.c_str()) < 0)
          if (chdir(bag_path_.c_str()) < 0)
          {
            ROS_WARN_STREAM(
                "Unable to cd into '"<<bag_path_<<"', using current directory.");
          }else
          {
            ROS_INFO_STREAM("Would store bag files to '"<<bag_path_<<"'"); 
          }
      else
      {
        ROS_INFO_STREAM("Would store bag files to '"<<req.bag_path<<"'");
      }
      setpgid(0, 0); // Required so the process group can be terminated
      execlp("sh", "sh", "-c",
             std::string("rosbag record ").append(req.rosbag_args).c_str(),
             NULL);
      ROS_INFO_STREAM("exec finished");
      break;
    }
    case -1: // Failed to fork
    {
      ROS_WARN("Unable to fork recording process.");
      return false;
    }
  }
  return true;
}

bool DataLogger::stop(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
  if (bag_process_pid_ != 0)
  {
    status_pub_.pubStopped();
    if (killpg(bag_process_pid_, SIGINT) == 0)
    {
      int status;
      waitpid(bag_process_pid_, &status, 0);
      ROS_INFO_STREAM("Stopped recording process, pid: "<<bag_process_pid_);
      bag_process_pid_ = 0;
      return true;
    }
    else
    {
      ROS_WARN("Could not SIGINT the child recording process.");
    }
  }
  else
  {
    ROS_WARN(
        "Stop data recording requested. Recording process is not running, can not stop.");
  }
  return false;
}
} // namespace data_logger
