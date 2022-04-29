#include <ros/ros.h>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <math.h>
#include <unistd.h>
#include <sensor_msgs/Image.h>

std::string camFile = "/dev/video0";
int width = 640;
int height = 480;
int fps = 30;

int main (int argc, char** argv){
    ros::init(argc, argv, "camera_stream_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(60);
    ros::param::get("/cam/file", camFile);
    ros::param::get("/cam/width", width);
    ros::param::get("/cam/height", height);
    ros::param::get("/cam/fps", fps);
    image_transport::ImageTransport it(nh);
    image_transport::Publisher cap_pub = it.advertise("/camera/image_raw", 1);
    cv::VideoCapture cap(camFile);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    cap.set(cv::CAP_PROP_FPS, fps);
    if (cap.isOpened() == false){
        ROS_ERROR_STREAM("Cannot open the video camera");
        return -1;
    } 
    while(ros::ok()){
        cv::Mat frame;
        if (!cap.read(frame)){
            ROS_ERROR_STREAM("Video camera is disconnected");
            return -1;
        }
        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg;
        std_msgs::Header header;
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);
        img_bridge.toImageMsg(img_msg);

        cap_pub.publish(img_msg);
        //loop_rate.sleep();
    }
}