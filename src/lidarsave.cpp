#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <cmath>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)
cv::VideoWriter video_writer;

static void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  Mat canvas(Size(500,500), CV_8UC3, Scalar(255,255,255));
  Mat rot = getRotationMatrix2D(Point(250,250), 0.5, 1.0);
  float distance = 25.0f;
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    printf("count : %d , [SLLIDAR INFO]: angle-distance : [%f, %f]\n", count , degree, scan->ranges[i]);
    float x = 250 + distance * scan->ranges[i] * cos(degree * CV_PI / 180.0f);
    float y = 250 + distance * scan->ranges[i] * sin(degree * CV_PI / 180.0f);
    drawMarker(canvas, Point(cvRound(x),cvRound(y)),Scalar(0,0,255),MARKER_SQUARE,2);
    getRotationMatrix2D(Point(250,250), 0.5, 1.0);
  }
  imshow("win", canvas);
  waitKey(1);

  video_writer.write(canvas);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sllidar_client");
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);
  video_writer.open("lidar.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(500, 500));
  rclcpp::spin(node);
  video_writer.release();
  rclcpp::shutdown();
  return 0;
}