# AROM

<img src="/DOC/graphics/Arom_logo_small.png" width="150">

[Video trailer](https://www.youtube.com/watch?v=syvk5YA4OgQ)

AROM (Autonomous robotic observatory manager) is set of open-source software for control and management robotic observatories. Whole software is built upon system for controlling robots - ROS (Robotic operation system). 

AROM software is designed for use with small (amateur) telescope and fully autonomous observatories. It is going to work on simple single-board computers souch as Odroid. Software should be able to monitor all telescope states which may affect observing quality. This conditions are weather, air-quality, observatory status, mount position and many other conditions.

### Introduction
There are few systems for controlling robotics telescopes. Some of them are commercial and so expensive. They are usually made bespoke for one assembly. There are few open-source (or free) systems which are not well working. Mostly there are unable to control observatory in full autonomous mode.

### Goals
The goals of this project are taken from the gained experience with observing and testing other systems.

##### Main goals are:
 * Minimal maintenance requirements
 * Independent on control computer
 * Service-lees observing (robotic observing)
 * Self-calibration, self-diagnostic
 * Price affordability
 * Simple and intuitive operation
 * Multiple telescope observing
 * Open-source software, easily expandable
