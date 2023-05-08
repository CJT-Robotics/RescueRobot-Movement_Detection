#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <joy_relay/CJT.h>

using namespace cv;
using namespace std;

// globale Variablen
Mat frame;
Mat grayFrame;
Mat prevFrame;
Mat diffFrame;
Mat threshFrame;
bool drawRect = false;
bool test = false;


// Funktion, um bewegte Objekte zu erkennen und einen Rahmen um sie zu zeichnen
void detectMotion(Mat currentFrame)
{
  cvtColor(currentFrame, grayFrame, COLOR_BGR2GRAY);
  GaussianBlur(grayFrame, grayFrame, Size(21, 21), 0);

  // Hintergrund berechnen
  if (prevFrame.empty())
  {
    grayFrame.copyTo(prevFrame);
  }

  absdiff(prevFrame, grayFrame, diffFrame);
  threshold(diffFrame, threshFrame, 25, 255, THRESH_BINARY);

  // Rahmen zeichnen
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(threshFrame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  // Polygondarstellung und rechteckige Begrenzungsrahmen erzeugen
  vector<vector<Point>> contoursPoly(contours.size());
  vector<Rect> boundRect(contours.size());

  for (size_t i = 0; i < contours.size(); i++)
  {
    if (contourArea(contours[i]) > 10)
    {
      Rect rect = boundingRect(contours[i]);
      //if (drawRect)
        //rectangle(currentFrame, rect, Scalar(0, 0, 255), 2);
    }
  }

  grayFrame.copyTo(prevFrame);

  for (size_t i = 0; i < contours.size(); i++)
  {
    approxPolyDP(Mat(contours[i]), contoursPoly[i], 3, true);
    boundRect[i] = boundingRect(Mat(contoursPoly[i]));
  }

  vector<vector<Point>> mergedContours;
  vector<Rect> mergedRects;
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
    if (contourArea(mergedContours[i]) > 10)
    {
      rectangle(currentFrame, mergedRects[i], Scalar(0, 0, 255), 2);
    }
  }
}

void buttonCallback(const joy_relay::CJT& msg) {
  string action = msg.action;
  string device = msg.device;

  if(device.compare("camera") == 0)  {
    if(action.compare("movement_detection") == 0) {
      test = !test;
    }
  }
}

// Callback-Funktion für den Empfang von Bildern
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  char c = waitKey(30);

  if(c == 'd') {
    test = !test;
  }

  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (test == true) {
      detectMotion(frame);
    }

    imshow("RoboCupGermany - CJT-Gymnasium - Movement Detection", frame);
    waitKey(1);
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
  namedWindow("RoboCupGermany - CJT-Gymnasium - Movement Detection");
  destroyWindow("RoboCupGermany - CJT-Gymnasium - Movement Detection");
  ros::spin();
  return 0;
}