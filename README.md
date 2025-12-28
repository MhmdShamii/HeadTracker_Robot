Wireless Head-Tracking Camera System (Arduino + ESP32-CAM)

This project is a wireless head-tracking camera system that translates human head movements into real-time camera motion. It uses an IMU-based transmitter worn on the head and a motorized receiver unit that physically moves a camera in pitch and yaw, while streaming live video.

System Overview

The system is split into two independent units:

1. Transmitter Unit

Arduino Uno

MPU6050 IMU (accelerometer + gyroscope)

NRF24L01 radio module

Calibration button

Status LEDs

The transmitter continuously reads head orientation (pitch & yaw) from the MPU6050.
On startup or button press, the system performs calibration to zero the reference angles.
Filtered and smoothed orientation data is sent wirelessly using the NRF24L01.

Transmitter LEDs indicate:

Radio errors or link loss

Calibration in progress

System ready

Packet transmission activity

2. Receiver Unit

Arduino Uno

NRF24L01 radio module

Servo motor (controls pitch)

Stepper motor (28BYJ-48) (controls yaw)

ESP32-CAM (live video streaming)

Power regulation with buck converter and capacitors

Status LEDs

The receiver listens for IMU data and converts it into physical camera movement:

Pitch → Servo motor

Yaw → Stepper motor

A startup self-test verifies motor operation, and safety logic centers the camera if the wireless signal is lost.

Power Architecture

Motors are powered directly from a battery pack to avoid brown-outs.

Logic components use regulated voltage via a buck converter.

Multiple bulk and decoupling capacitors are placed near high-current devices (ESP32-CAM and motors) to stabilize voltage during current spikes.

All grounds are shared to ensure signal integrity.

This power design prevents resets, reduces jitter, and stabilizes camera streaming.

Key Features

Wireless head-tracking with real-time response

Robust RF communication with timeout safety

IMU calibration via button

Motor self-test on startup

Smooth motion using filtering and step control

Live video streaming via ESP32-CAM

Fully battery-powered system

LED-based diagnostics for debugging and status monitoring

Applications

FPV and telepresence systems

Robotics vision platforms

Remote inspection tools

Camera stabilization experiments

Human-machine interaction research

Conclusion

This project demonstrates the integration of sensors, wireless communication, motor control, power electronics, and real-time video streaming into a single embedded system. Special attention was given to power stability, reliability, and fault handling, making the system practical and extensible for future robotic and vision-based applications.
