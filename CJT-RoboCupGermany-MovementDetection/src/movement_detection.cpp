#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <joy_relay/CJT.h>

// globale Variablen
cv::Mat frame;
cv::Mat grayFrame;
cv::Mat prevFrame;
cv::Mat diffFrame;
cv::Mat threshFrame;
bool drawRect = false;
bool test = false;


// Funktion, um bewegte Objekte zu erkennen und einen Rahmen um sie zu zeichnen
void detectMotion(cv::Mat currentFrame)
{
  cv::cvtColor(currentFrame, grayFrame, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(grayFrame, grayFrame, cv::Size(21, 21), 0);

  // Hintergrund berechnen
  if (prevFrame.empty())
  {
    grayFrame.copyTo(prevFrame);
  }

  cv::absdiff(prevFrame, grayFrame, diffFrame);
  cv::threshold(diffFrame, threshFrame, 25, 255, cv::THRESH_BINARY);

  // Rahmen zeichnen
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(threshFrame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Polygondarstellung und rechteckige Begrenzungsrahmen erzeugen
  std::vector<std::vector<cv::Point>> contoursPoly(contours.size());
  std::vector<cv::Rect> boundRect(contours.size());

  for (size_t i = 0; i < contours.size(); i++)
  {
    if (cv::contourArea(contours[i]) > 10)
    {
      cv::Rect rect = cv::boundingRect(contours[i]);
      //if (drawRect)
        //cv::rectangle(currentFrame, rect, cv::Scalar(0, 0, 255), 2);
    }
  }

  grayFrame.copyTo(prevFrame);

  for (size_t i = 0; i < contours.size(); i++)
  {
    cv::approxPolyDP(cv::Mat(contours[i]), contoursPoly[i], 3, true);
    boundRect[i] = cv::boundingRect(cv::Mat(contoursPoly[i]));
  }

  std::vector<std::vector<cv::Point>> mergedContours;
  std::vector<cv::Rect> mergedRects;
  int mergeThreshold = 250;

  for (size_t i = 0; i < contours.size(); i++)
  {
    bool merged = false;
    for (size_t j = 0; j < mergedContours.size(); j++)
    {
      int dx = boundRect[i].x - mergedRects[j].x;
      int dy = boundRect[i].y - mergedRects[j].y;
      int dw = boundRect[i].width - mergedRects[j].width;
      int dh = boundRect[i].height - mergedRects[j].height;
      if (abs(dx) < mergeThreshold && abs(dy) < mergeThreshold && abs(dw) < mergeThreshold && abs(dh) < mergeThreshold)
      {
        mergedContours[j].insert(mergedContours[j].end(), contours[i].begin(), contours[i].end());
        mergedRects[j] = mergedRects[j] | boundRect[i];
        merged = true;
        break;
      }
    }
    if (!merged)
    {
      mergedContours.push_back(contours[i]);
      mergedRects.push_back(boundRect[i]);
    }
  }

  // Rahmen zeichnen
  for (size_t i = 0; i < mergedRects.size(); i++)
  {
    if (cv::contourArea(mergedContours[i]) > 10)
    {
      cv::rectangle(currentFrame, mergedRects[i], cv::Scalar(0, 0, 255), 2);
    }
  }
}

void buttonCallback(const joy_relay::CJT& msg) {
  std::string action = msg.action;
  std::string device = msg.device;

  

  


  if(device.compare("camera") == 0)  {
    if(action.compare("movement_detection") == 0) {
      test = !test;
    }
  }
}

// Callback-Funktion für den Empfang von Bildern
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  char c = cv::waitKey(30);

  if(c == 'd') {
    test = !test;
  }

  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (test == true) {
      detectMotion(frame);
    }

    cv::imshow("RoboCupGermany - CJT-Gymnasium - Movement Detection", frame);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imsub = it.subscribe("/cjt/image_raw", 1, imageCallback, image_transport::TransportHints("compressed"));
  ros::Subscriber btnsub = nh.subscribe("/cjt/input", 1, buttonCallback);
  cv::namedWindow("RoboCupGermany - CJT-Gymnasium - Movement Detection");
  ros::spin();
  cv::destroyWindow("RoboCupGermany - CJT-Gymnasium - Movement Detection");
  return 0;
}