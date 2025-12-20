##🚀 Current Status (Stage 1 — December 20, 2025)
We are in Stage 1: Basic Control and Simulation Validation.
Achievements

##Hardware integration complete: STM32 Nucleo-F401RE (low-level control) + Raspberry Pi (high-level processing).
Embedded firmware (Mbed OS) deployed and stable.
Reliable serial communication established between Nucleo and Raspberry Pi.
Sensors active: IMU (roll, pitch, yaw, accelerations) and battery monitoring with safety features.
Physical car responds to manual commands.

##Key Focus
We are primarily developing and testing control logic in a Gazebo simulator with ROS. This approach ensures precise, repeatable, and safe validation before full physical deployment.
Demonstration Video We have recorded a video showcasing highly accurate control in the simulator (synced with initial physical tests): precise speed holding, steering, timed movements, and clean sensor logging.

##📈 Performance & Control System
Performance

Simulator: Extremely accurate and precise

  * Speed accuracy: ±1 mm/s
  * Steering: exact angles, zero latency
  * Perfect repeatability for trajectory analysis

##Physical Car: Solid baseline

  * Stable runs at 300–400 mm/s
  * Quick steering response
  * Minor real-world deviations (to be calibrated next)

##IMU data reliable for orientation and stability feedback.

##How We Control the Car
##Primary Method (Current Stage)

##ROS + Gazebo simulator

  * Commands published via ROS topics (e.g., /cmd_vel, custom timed-move topics)
  * Simulated physics provides perfect execution
  * All runs recorded as ROS bag files for analysis
Physical Backup

##Direct serial commands to Nucleo-F401RE: #speed:200;;\r\n → 200 mm/s #steer:100;;\r\n → +10° steering #vcd:150;50;200;;\r\n → 150 mm/s, +5° steer, 20 seconds #imu:1;;\r\n → Enable IMU data stream

##Processing Flow

##High-level system (Raspberry Pi / ROS master) computes actions.
Sends commands (ROS topics or serial).
Executor (simulator or Nucleo) controls motors.
Sensors (IMU, battery, etc.) return feedback.
Closed-loop adjustments + logging.

##IMU data is processed for yaw correction, tilt detection, and future odometry fusion.
🔮 Future Plans
We are building a modular, scalable system centered on ROS/ROS2.
Short-Term (Jan–Feb 2026)

##Port validated simulator controls to the physical car.
Implement sensor fusion (IMU + wheel encoders).
Begin basic lane detection testing.

##Mid-Term (Mar–May 2026)

##Full ROS2 stack on Raspberry Pi with separate nodes for perception, planning, and control.
Implement core algorithms:

  * Lane following: Pure Pursuit / Stanley controller
  * Traffic sign detection: CNN-based models
  * Global planning: A* / Hybrid A*
  * Local avoidance: Dynamic Window Approach (DWA) or Model Predictive Control (MPC)
  * V2I communication handling
Long-Term (Competition Preparation)

##End-to-end autonomous pipeline.
##Advanced navigation with IMU-camera fusion and SLAM.
##Robust behavior planning for intersections, parking, and complex scenarios.
##Extensive real-track testing and iterative improvement.

##All progress, code, ROS packages, documentation, and new demo videos will be updated here regularly.
🙏 Acknowledgments
Special thanks to:

##Bosch Engineering Center Cluj and the BFMC organizers
Our university mentors and advisors
Open-source communities and previous BFMC teams for inspiration and resources

##Team Autonomists — Pushing the limits of autonomous driving, one lap at a time! 🚀 
