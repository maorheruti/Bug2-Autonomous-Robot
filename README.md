# Bug2 Autonomous Robot

A clean public showcase of an ESP32-based Bug2 navigation robot. The project evolved from an earlier RobotChaser target-following system into an onboard autonomous navigation platform using pure pursuit, Bug2-style wall following, matrix time-of-flight sensing, and a browser telemetry dashboard.

## Project Motivation

RobotChaser used OptiTrack pose data with most navigation logic running on a PC. The PC computed pure-pursuit motion commands and sent them to the ESP32 robot.

The Bug2 project moved the main navigation decisions onboard. OptiTrack still provides pose data through the PC, but the ESP32 now handles go-to-goal behavior, obstacle response, wall following, M-line leave/rejoin behavior, recovery logic, and telemetry. This made the robot a practical embedded experimentation platform rather than only a remote-commanded target follower.

## Architecture Overview

The high-level architecture is:

```text
OptiTrack -> PC pose stream -> onboard ESP32 Bug2 navigation -> motor and servo control
```

The ESP32 firmware combines:

- pure-pursuit go-to-goal motion,
- Bug2-style wall following,
- M-line leave/rejoin behavior,
- matrix time-of-flight obstacle sensing,
- runtime tuning support,
- a browser-based telemetry and obstacle visualization dashboard.

## Features

- Onboard ESP32 navigation and decision-making.
- Pure pursuit for goal-directed motion.
- Bug2 wall-following behavior with recovery logic.
- VL53L5CX/VL53L7CX matrix ToF sensor integration.
- Telemetry dashboard with robot trajectory and obstacle visualization.
- Runtime tuning and debugging endpoints.
- Demonstrated wall following and return-to-goal behavior.

## Hardware

- ESP32-based mobile robot.
- Ackermann steering with servo control.
- DC motor drive.
- OptiTrack pose input through a PC pose stream.
- Three matrix time-of-flight sensors:
  - left VL53L5CX,
  - middle VL53L7CX,
  - right VL53L5CX.

## Demonstration Videos

- [Bug2 reaching goal 1](assets/videos/Bug2_reaching_goal_1.mp4)  
  Demonstrates goal-directed continuation and return-to-goal behavior.

- [Bug2 reaching goal 2](assets/videos/Bug2_reaching_goal_2.mp4)  
  Shows an additional reaching-goal run from the Bug2 test sequence.

- [Bug2 wall-following overtime testing](assets/videos/Bug2_wall_following_overtime_testing.mp4)  
  Demonstrates longer wall-following behavior and tuning-oriented testing.

## Engineering Report

The professor-facing engineering reflection is available here:

- [BUG2_PROJECT_REPORT_EXPORT.pdf](docs/BUG2_PROJECT_REPORT_EXPORT.pdf)
- [BUG2_PROJECT_REPORT_EXPORT.md](docs/BUG2_PROJECT_REPORT_EXPORT.md)

## Current Status

The core system is implemented and demonstrated: onboard navigation, pure pursuit, Bug2-style wall following, sensor redesign, telemetry, and return-to-goal behavior are represented in the report and media.

The robot is still an actively tuned research/engineering platform. Broader robustness testing across repeatable obstacle layouts and scenarios remains future work.
