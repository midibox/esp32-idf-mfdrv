# MBHP_MF_NG for ESP32

## Introduction

This application is originated from the PIC based MBHP_MF_NG project: http://www.ucapps.de/mbhp_mf_ng.html

Currently used as a "Proof of Concept" for ESP32 based motorfader control.

Advantage: due to the dedicated PWM outputs of ESP32 we get much more accurate control over the motors.

More documentation to follow...


## Important

Please optimize the app configuration with "idf.py menuconfig":

* Compiler Options->Optimization Level: set to -Os (Release)
* Component Config->ESP32 Specific: set Minimum Supported ESP32 Revision to 1 (if you have a newer device...)
* Component Config->ESP32 Specific: set CPU frequency to 240 MHz
* Component Config->Log output: set to Warning (or use Info if you want to follow the data flow - but it will decrease performance!)
