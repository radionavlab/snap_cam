// Author: Tucker Haydon

#include <ios>
#include <nav_msgs/Odometry.h>

#include "metadata_logger.h"
#include "odometry_buffer.h"

// Extern Variable
// OdometryBuffer odometry_buffer;

MetadataLogger::MetadataLogger(const std::string& log_directory_path, const std::string& log_file_name) {
  this->log_directory_path_ = log_directory_path;
  this->log_file_name_ = log_file_name;
  this->log_file_path_ = this->log_directory_path_ + "/" + this->log_file_name_;

  this->CreateLogFile();
};

MetadataLogger::~MetadataLogger() {
  this->output_file_ptr_->close();
}

void MetadataLogger::CreateLogFile() {
  // Open and clear file
  this->output_file_ptr_ = new std::ofstream(this->log_file_path_, std::ios_base::trunc);

  // Set the precision
  *(this->output_file_ptr_) << std::setprecision(10);

  // Write header
  *(this->output_file_ptr_) << "# ImageName Pose{Position{X Y Z} Orientation{W X Y Z}} PoseCovariance{36 elements, {position,orientation, row-major}" << std::endl;
};

void MetadataLogger::LogMetadata(const std::string& row_name) {
  // Copy most recent pose estimate
  nav_msgs::Odometry odom = odometry_buffer.GetOdometryMsg();
  std::cout << odom.pose.pose.position.x << std::endl << std::endl;

  // Write row name
  *(this->output_file_ptr_) << row_name << " ";

  // Write pose
  *(this->output_file_ptr_) << 
    odom.pose.pose.position.x << " " << 
    odom.pose.pose.position.y << " " << 
    odom.pose.pose.position.z << " " << 
    odom.pose.pose.orientation.w << " " << 
    odom.pose.pose.orientation.x << " " << 
    odom.pose.pose.orientation.y << " " << 
    odom.pose.pose.orientation.z << " ";

  // Write pose covariance
  for(const auto ROS_Float: odom.pose.covariance) {
    *(this->output_file_ptr_) << ROS_Float << " ";
  }

  // New line
  *(this->output_file_ptr_) << std::endl;

  // Flush buffer
  this->output_file_ptr_->flush();
};

