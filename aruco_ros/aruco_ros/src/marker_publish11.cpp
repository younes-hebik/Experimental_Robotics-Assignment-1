#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <algorithm>

class ArucoMarkerPublisher
{
private:
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  std::vector<aruco::Marker> markers1_;
  std::vector<aruco::Marker> sorted_markers_;  
  aruco::CameraParameters camParam_;
  double marker_size_;
  bool useCamInfo_;
  bool all_markers_detected_;  
  int current_marker_index_;  
  int k = 0;  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  ros::Publisher vel_pub_;

  cv::Mat inImage_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true), all_markers_detected_(false), current_marker_index_(0)
  {
    image_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
    nh_.param<double>("marker_size", marker_size_, 0.1);  
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;

      if (!all_markers_detected_)
      {
        
        std::vector<aruco::Marker> currentMarkers;
        mDetector_.detect(inImage_, currentMarkers, camParam_, marker_size_, false);

        for (const auto& marker : currentMarkers)
        {
          auto it = std::find_if(markers_.begin(), markers_.end(), 
              [&marker](const aruco::Marker& m) { return m.id == marker.id; });

          if (it == markers_.end()) {
            markers_.push_back(marker);
           // std::cout << "Detected marker with ID: " << marker.id << std::endl;
          }
        }

        
        for (const auto& marker : markers_)
        {
          marker.draw(inImage_, cv::Scalar(0, 0, 255), 2);
        }

      
        geometry_msgs::Twist vel;
        vel.angular.z = 0.5;
        vel_pub_.publish(vel);

        
        if (markers_.size() >= 3)  // Assume we expect 3 markers
        {
          all_markers_detected_ = true;
          sorted_markers_ = markers_;
          std::sort(sorted_markers_.begin(), sorted_markers_.end(), [](const aruco::Marker& a, const aruco::Marker& b) {
              return a.id < b.id;
          });
          std::cout << "All markers detected and sorted by ID." << std::endl;
        }
      }
      else
      {
        // Second pass: Detect markers and publish them in sorted order
        if (k < sorted_markers_.size())
        {
          markers1_.clear();
          mDetector_.detect(inImage_, markers1_, camParam_, marker_size_, false);

         
          for (const auto& marker : markers1_)
          {
            if (marker.id == sorted_markers_[k].id)
            {
              std::cout << "Found marker with ID: " << marker.id << std::endl;
              // Draw the marker with a circle
              cv::circle(inImage_, cv::Point(marker.getCenter().x, marker.getCenter().y), 30, cv::Scalar(255, 0, 0), 2);
              // Publish the image with the marked marker
              cv_bridge::CvImage out_msg;
              out_msg.header.stamp = ros::Time::now();
              out_msg.encoding = sensor_msgs::image_encodings::RGB8;
              out_msg.image = inImage_;
              image_pub_.publish(out_msg.toImageMsg());

              // Remove the marker from the sorted list after publishing
              sorted_markers_.erase(sorted_markers_.begin());
              //k++;  // Move to the next marker in the sorted order
              break;
            }
          }

         
          geometry_msgs::Twist vel;
          vel.angular.z = 0.5;
          vel_pub_.publish(vel);
        }
        else
        {

          geometry_msgs::Twist vel;
          vel.angular.z = 0.0;
          vel_pub_.publish(vel);
          std::cout << "Finished publishing all markers in sorted order." << std::endl;
        }
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");
  ArucoMarkerPublisher node;
  ros::spin();
}

