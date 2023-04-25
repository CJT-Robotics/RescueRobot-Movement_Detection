# RescueRobot-Movement_Detection
This Programm uses a rostopic, to track movement of a webcam. Around the detected motion there will be placed a colored bouning box.

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
