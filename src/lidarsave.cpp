#include "rclcpp/rclcpp.hpp"// ROS2의 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/laser_scan.hpp"// 라이다 센서 메시지를 포함
#include <math.h>
#include <cmath>// 수학 함수들을 사용하기 위한 헤더 파일
#include "opencv2/opencv.hpp"// OpenCV 라이브러리를 포함
using namespace std;
using namespace cv;

// 라디안을 각도로 변환하는 매크로 정의
#define RAD2DEG(x) ((x)*180./M_PI)

VideoWriter video_writer;//비디오를 저장

// 라이다 데이터를 처리하는 콜백 함수
static void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment; // 라이다 스캔 데이터의 수를 계산
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  Mat canvas(Size(500,500), CV_8UC3, Scalar(255,255,255));// 라이다 데이터를 시각화할 canvas를 초기화

  float distance = 25.0f; // 일정한 거리를 정의

    // 라이다 데이터를 반복하면서 처리
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // 각도를 계산
    printf("count : %d , [SLLIDAR INFO]: angle-distance : [%f, %f]\n", count , degree, scan->ranges[i]);// 화면에 각도 출력

    // 캔버스 상에 그리기 위한 좌표를 계산
    float x = 250 + distance * scan->ranges[i] * cos(degree * CV_PI / 180.0f);
    float y = 250 + distance * scan->ranges[i] * sin(degree * CV_PI / 180.0f);

    // 계산된 좌표에 마커를 그림
    drawMarker(canvas, Point(cvRound(x),cvRound(y)),Scalar(0,0,255),MARKER_SQUARE,2);
    getRotationMatrix2D(Point(250,250), 0.5, 1.0);
  }
  imshow("win", canvas); // 화면에 출력
  waitKey(1);

  video_writer.write(canvas); // 화면에 출력된 canvas를 비디오로 저장
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // ROS2 노드를 초기화
  auto node = rclcpp::Node::make_shared("sllidar_client");// "sllidar_client"라는 이름의 노드를 생성
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb); // "scan" 토픽에 서브스크립션을 생성
  video_writer.open("lidar.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, cv::Size(500, 500)); // 비디오 저장 형식 지정
  rclcpp::spin(node);
  video_writer.release();
  rclcpp::shutdown();
  return 0;
}
