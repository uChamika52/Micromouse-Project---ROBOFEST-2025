# Micromouse-Project-ROBOFEST-2025
Maze-solving Micromouse for ROBOFEST 2025 | ESP32-based robot with ToF sensing and Flood-fill navigation.

Autonomous maze-solving Micromouse robot developed for SLIIT ROBOFEST 2025 (University Category). Built using ESP32, VL53L0X ToF sensors, and N20 motors with encoders, implementing the Flood Fill algorithm for intelligent navigation and optimized run times.

## Table of Contents
1. [Introduction](#introduction)
2. [Project Structure](#project-structure)
3. [Hardware & Software Requirements](#hardware--software-requirements)
4. [Meet the Team](#meet-the-team)

---

## Introduction

This repository documents our **Micromouse robot**, designed for the **University Category of SLIIT ROBOFEST 2025**.  
Our goal was to build a fully autonomous robot capable of **navigating and solving a 16×16 maze** using the **Flood Fill algorithm**, adhering strictly to the official technical specifications of the competition.

The Micromouse was built around the **ESP32 Wroom 32** platform, integrating **VL53L0X ToF sensors**, **N20 motors with encoders**, and **MX1616 motor driver** to achieve precise control and intelligent pathfinding.

---

## Project Structure

/Docs → Pinouts, wiring diagrams, wall layout references
/Code → All main and test Arduino sketches

---

## Hardware & Software Requirements

### Hardware:
- ESP32 Wroom 32 Development Board  
- VL53L0X Time-of-Flight Sensors (x3)  
- MX1616 Motor Driver  
- N20 Motors with Encoders (x2)  
- AMS1117 & 7805 Voltage Regulators  
- 3.7V 800mAh LiPo Batteries  

### Software:
- Arduino IDE 
- Required Libraries:
  - `Adafruit_VL53L0X.h`

---

## Meet the Team

This project was developed by a team of Electronic and Telecommunication Engineering undergraduates from the University of Sri Jayewardenepura for ROBOFEST 2025.

Ushan Karunarathna
Sasindu Perera 
Achinthya Bimsara
Madhuka Dias
Laksith Kahatapitiya
