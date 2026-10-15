# Dual Swing Gate Control w/ RF

This project implements a dual-swing gate control system using a radio-frequency (RF) remote and ESP32/Arduino-compatible hardware. It is designed to automate a pair of swing gates with limit-switch feedback, obstacle detection, manual override, and RF control.

## Features

- Dual gate control for left and right gates
- RF remote activation using a receiver module
- Gate movement with configurable open/close timing
- Limit switch support for each gate
- Obstacle detection via IR or safety sensor input
- Manual push-button control
- Real-time status reporting over serial monitor
- Optional safety and lock logic for gate positioning

## Hardware

The project is built around an ESP32-compatible microcontroller and includes:

- ESP32 dev board
- RF receiver module
- Gate actuator relay board
- Limit switches (open/close for each gate)
- Obstacle detection sensor(s)
- Manual control buttons or switches
- Power supply and wiring as required by the motor driver
