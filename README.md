# RescueRobot-Movement_Detection
This Programm uses a rostopic, to track movement of a webcam. Around the detected motion there will be placed a colored bouning box.

# Tutorial
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
  3. Build the workspace.
  ```bash
  cd ..
  catkin_make
  ```
  (4.) If you don't use our joy_relay [1], you have to manipulate our code.
  ```cpp
  //Missing
  ```
