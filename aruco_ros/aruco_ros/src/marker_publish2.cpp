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
#include <std_msgs/Float64.h>
class ArucoMarkerPublisher
{
private:

  struct iden{
  
    aruco:: Marker  marker;
    int   pos; // the order of the marker in a circle 
  
  
  };
  
  std::vector<iden> sorted_iden_;
  int temp_pos; 
  int c1=0;
  int rot_L{},rot_R{};

   
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
  int c=0;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher camera_vel_pub_;
  cv::Mat inImage_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true), all_markers_detected_(false), current_marker_index_(0)
  {
    image_sub_ = it_.subscribe("/robot/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
   camera_vel_pub_ = nh_.advertise<std_msgs::Float64>("/robot/camera_joint_velocity_controller/command", 10);


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
            c++; // increament the counter of the position oredr
            iden ident1={marker,c};
            sorted_iden_.push_back(ident1);
           std::cout << "Detected marker with ID: " << marker.id << std::endl;
          }
        }

        
        for (const auto& marker : markers_)
        {
          marker.draw(inImage_, cv::Scalar(0, 0, 255), 2);
        }

    


	std_msgs::Float64 velocity_msg;
        velocity_msg.data = 1.0; // Set desired velocity here
        camera_vel_pub_.publish(velocity_msg);
        
        if (markers_.size() >= 5)  // Assume we expect 4 markers
        {
          all_markers_detected_ = true;
          sorted_markers_ = markers_;
          std::sort(sorted_markers_.begin(), sorted_markers_.end(), [](const aruco::Marker& a, const aruco::Marker& b) {
              return a.id < b.id;
          });
          std::cout << "All markers detected and sorted by ID." << std::endl;
          
          
          
             
         std::sort(sorted_iden_.begin(), sorted_iden_.end(), [](const iden& a, const iden& b) {
        return a.marker.id < b.marker.id;
    });

          
          
        }
        
     


      }
      else
      {
        // Second pass: Detect markers and publish them in sorted order
        if (k < sorted_iden_.size())
        {
          markers1_.clear();
          mDetector_.detect(inImage_, markers1_, camParam_, marker_size_, false);

         
          for (const auto& marker : markers1_)
          {
            if (marker.id == sorted_iden_[k].marker.id)
            {
              std::cout << "Found marker with ID: " << marker.id << std::endl;
              // Draw the marker with a circle
             // cv::circle(inImage_, cv::Point(marker.getCenter().x, marker.getCenter().y), 30, cv::Scalar(255, 0, 0), 4);
              
                              marker.draw(inImage_, cv::Scalar(0, 0, 255), 2);
              
	  
  
  
              // Publish the image with the marked marker
              cv_bridge::CvImage out_msg;
              out_msg.header.stamp = ros::Time::now();
              out_msg.encoding = sensor_msgs::image_encodings::RGB8;
              out_msg.image = inImage_;
              image_pub_.publish(out_msg.toImageMsg());
              temp_pos= sorted_iden_[k].pos;
	      c1=1;
              // Remove the marker from the sorted list after publishing
              sorted_iden_.erase(sorted_iden_.begin());
              
              //k++;  // Move to the next marker in the sorted order
              break;
            }
          }

         if(c1==0){
         
       
          
          
        std_msgs::Float64 velocity_msg;
        velocity_msg.data = 1.0; // Set desired velocity here
        camera_vel_pub_.publish(velocity_msg);
          
          
          }
          else{
			// Calculate rotation distances using ternary operators
		rot_L = (sorted_iden_[k].pos - temp_pos > 0) 
		? (sorted_iden_[k].pos - temp_pos) 
		: (5 - std::abs(sorted_iden_[k].pos - temp_pos));

	rot_R = (sorted_iden_[k].pos - temp_pos > 0) 
		? (5 - std::abs(sorted_iden_[k].pos - temp_pos)) 
		: (sorted_iden_[k].pos - temp_pos);

	// Decide rotation direction based on distances using a ternary operator
	
	
	std_msgs::Float64 velocity_msg;
        velocity_msg.data = (rot_L < rot_R) ? 1.0 : -1.0;
        camera_vel_pub_.publish(velocity_msg);

          }
          
        }
        else
        {
       
          
          
          std_msgs::Float64 velocity_msg;
        velocity_msg.data = 0; // Set desired velocity here
        camera_vel_pub_.publish(velocity_msg);
          
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

