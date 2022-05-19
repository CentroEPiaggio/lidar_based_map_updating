#include <ros/ros.h>
#include <ros/rate.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

sensor_msgs::ImageConstPtr imagePT;

bool flag_image = false;
void imageCallback(const sensor_msgs::CompressedImage& msg)
{
    cv::Mat image = cv::imdecode(cv::Mat(msg.data),1);
    sensor_msgs::ImagePtr img_msg_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    imagePT.reset(new sensor_msgs::Image(*img_msg_msg));
    flag_image = true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "image_publisher");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/robot/front_rgbd_camera/rgb/image_raw/compressed", 10, &imageCallback);
     ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("/image", 1);


  sensor_msgs::ImagePtr img_msg;
    cv::Mat image_new;
  
  ros::Rate r(100.0);
  while(n.ok()){
    if(flag_image){
    cv_bridge::toCvCopy(imagePT, sensor_msgs::image_encodings::BGR8)->image.copyTo(image_new);

    img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image_new).toImageMsg();

    image_pub.publish(img_msg);
    }
		ros::spinOnce();

    r.sleep();
  }
}
