# RescueRobot-Movement_Detection
This Programm uses a rostopic, to track movement of a webcam. Around the detected motion there will be placed a colored bouning box. You can switch between the normal camera and the movement detection by pressing the button 'c' one time. If you repeat that, the image just switch back.

## Tutorial
To set up this Package for your own Robot, there is one possibility.

  1. Direct to your catkin-workspace.
  ```bash
  cd catkin_ws/
  cd src/
  ```
  2. Clone this repository.
  ```bash
  git clone https://github.com/CJT-Robotics/RescueRobot-Movement_Detection.git
  ```
  (3.1) If you don't use our joy_relay [1], you have to manipulate our code. Remove this method.
  ```cpp
  void buttonCallback(const joy_relay::CJT& msg) {
    std::string action = msg.action;
    std::string device = msg.device;
    if(device.compare("camera") == 0)  {
      if(action.compare("movement_detection") == 0) {
        test = !test;
      }
    }
  }
  ```
  (3.2) Remove the btnSubscriber.
  ```cpp
  ros::Subscriber btnsub = nh.subscribe("/cjt/input", 1, buttonCallback);
  ```
4. Build your catkin_workspace.
  ```bash
  cd build/
  catkin_make
  ```
## Dokumentation
 ### The main method:
  ```cpp
  ros::init(argc, argv, "image_subscriber");
  ```
  This initialize the ROS-Package. Here for the name you can choose your own name, but pay attention that this name could only used one time. If there is an other Package running with the same node-name, both will be crash.
  
  ```cpp
  image_transport::ImageTransport it(nh);
  ```
  This will call the ImageTransport constructor.
  
  ```cpp
  image_transport::Subscriber imsub = it.subscribe("/cjt/image_raw", 1, imageCallback, image_transport::TransportHints("compressed"));
  ```
  This subscribe the published image (Your Webcam or something else). You have to set your topic name to the one, you want to subscribe. Tipp to get the topic name: ```rostopic list``` imageCallback is the method to get the image and deal with it. If your topic supports compressed images, you need "image_transport::TransportHints("compressed")" because you can't subscribe the compressed topic directly.
  
  ```cpp
  cv::namedWindow("RoboCupGermany - CJT-Gymnasium - Movement Detection");
  ```
  This is used for creating a window, later you put in your image.
  
  ```cpp
  ros::spin();
  ```
  This is just for the subscriber and the threads.
