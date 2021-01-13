# Roadmap

## Completed 

- Set up Jetson Nano with ROS
- Test Google Cartographer with RPLidar
- Complete CAD
- Laser cut robot parts

## Week 1-2

**Software**
- Set up headless Jetson Nano for wireless control so we don't need to plug in monitor and keyboard/mouse **(DONE)**

**Electronics**
- Power motors w/ motor driver, Arduino, and 12V battery **(DONE)**
- Establish link between Arduino & ROS@Jetson Nano
- Move motors: ROS@Jetson Nano -> Arduino -> Motor Driver -> Motor
- Set up motor rotary encoders to send odometry to ROS **(NO NEED)**

**Assembly**
  - Once electronics is completed, assemble everything. **(DONE)**

**Testing**
  - Test Google Cartographer to map a room **(HALF DONE)**

## Week 2-3

Our contribution to SLAM starts here.

**Obstacle Avoidance System (Small Obstacles)**
- Establish link from ultrasonic sensor to ROS@Jetson Nano
- Write simple algorithm to avoid small obstacles
  - Detect close objects with ultrasonic sensor
  - Interrupt Cartographer SLAM
  - Use array of ultrasonic sensors to estimate size of object
  - Annotate object on Map
  - Use ultrasonic sensors and feedback loop to navigate around obstacle
  - Reinstate Cartographer SLAM

**Obstacle Avoidance System (Large Obstacles)**
- Write an algorithm to quantify SLAM Localization error with Cartographer
  - Because Cartographer does not have one built in
- Write algorithm to detect probability of obstacle using calculated SLAM Localization error and RPLidar data
- 2 Ways to go from here:
  - Ignore obstacle and move around it
  - OR if obstacle is too big and affects localization performance too much, force remap of environment with Cartographer

**Integration**
- Wrap algorithms into ROS packages
- Integrate algorithms with our Robot

**Testing**
- Make sure everything works

## Week 3-4

**Results**
- Set up experiments and quantifiable tests
- Quantify results
- Identify areas for improvements

**Iteration**
- Work on previously identified improvements

**Extra Features?**
- Integrate a camera and ML object detection to be able to identify the type of object
  - Can be used to add more information to the obstacle annotation on map
  - Can be used for decision making? - i.e.: Cardboard box -> instruct robot drive though it; Heavy rock -> instruct robot navigate around it

## Week 5-6

Midterms week, probably will be busy with other things but will still work on FYP.

**Extra Features?:**
- Moving obstacles
  - Make obstacle detection algorithm more robust and can now detect, annotate, and navigate around **moving obstacles**.
- Will think of more features to add.

**Results**
- Continue to set up experiments and quantifiable tests
- Quantify results

## Week 7+

Ideally will finish all development by week 7.

**Report**
- Start writing final report