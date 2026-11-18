# Changelog

## v1.2.0 (2026)

- Fix: use pymavlink's `common` dialect. From pymavlink 2.4.49 the `standard` dialect no longer has the mission / RC / ATTITUDE / GLOBAL_POSITION_INT / ODOMETRY messages, so both nodes failed on current installs.
- Fix (gcsdatahandlernode): invalid or incomplete MAVLink frames no longer crash the node.
- Fix (gcsdatahandlernode): mission upload — a new `MISSION_COUNT` restarts the upload, `count 0` is acknowledged, non-waypoint items advance the sequence, duplicate items are re-requested, `MISSION_CLEAR_ALL` drops partial waypoints.
- Fix (huskydatahandlernode): the clock jump callback called an undefined `reset_state()`; it now clears the GPS/IMU sync queues and the TF buffer.
- Package: MIT `LICENSE`, real description/maintainer/license in `package.xml` and `setup.py`, missing ROS dependencies declared, version 1.2.0.

## v1.1.0 (2025-11-20)

- 2D SLAM 관제를 위한 Odometry 연계 기능 추가
- 유연한 구조를 위해 ros2 launch 구조 적용

## v1.0.0 (2024-12-03)

- 충북 테크노파크에서 테스트를 수행한 첫 안정화 버전
