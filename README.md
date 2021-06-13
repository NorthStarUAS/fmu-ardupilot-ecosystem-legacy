# Rice Creek FMU

This is an ardupilot/chibios app for building the firmware that serves
as the heart of a Rice Creek UAV autopilot.

This README has not been fully updated for the ardupilot/chibios port,
so much of it is probably wrong or terribly out dated.  The code is
also just at a checkpoint state ... still very much a work in
progress.

## NOTES:

* AP_HAL appears (based on timing packets on the receiving side of the
  pipe) to only service the serial/uart output at 50hz.  This
  effectively limits the update rate with a host computer.  It may
  mean that inner loop pid's will need to be run on the FMU, not on
  the host when faster update rates area needed.

## Some major bullet point todo list items:

- figure out/test airdata connections (no external port on pixracer?)
- decide how best to detect and choose when multiple imu and compass
  are available.
- look through config messages and see what needs to be changed for
  the new system. (new rc-flight driver to match this updated so we
  can continue to support the goldy3 variant separately as long as we
  have that hardware in play.)
* sort out how I really want to organize classes vs. singletons
  vs. global instances vs. namespace vs. cross-wired interdependencies
  between everything.  I have some messy mixes of these due to deep
  dive, sprint development dynamics.
  * notice that statically allocating big things avoids potentially
    blowing up the stack space on some super cramped boards.
* look carefully at what is in setup_board.h and where this is
  #included all about
x figure out led's (AP_Notify)
x the_gps and the_imu are not great names. (_mgr is also not great,
  but I've used it before and I hate it less ...)
x do a round of file renaming, or figure out how to orgainize code in
  subdirs or some of both.

Rc-fmu turns an inexpensive teensy board into a sensor collector,
attitude determination system, communications hub, and servo
controller.  It is not yet a full fledged autopilot itself, but
designed to pair with a host linux board (such as a beaglebone or
raspberry pi) for all the higher level autopilot functions.  It
supports the mpu9250 imu, ublox8 gps, bme280/bmp280 pressure sensors,
sbus receiver, and attopilot volt/amp sensor.  It also supports an
external airdata systems via the i2c bus.

![prototype](images/IMG_20191118_064616925.jpg "Prototype board")

As of version 4, a high accurancy 15-state EKF has been added for
precision attitude and lcoation estimate. It is designed to work
exceptionally well for outdoor dynamic systems such as fixed wing
aircraft.

Rc-fmu is one component of a research grade autopilot system that
anyone can assemble with basic soldering skills.  Altogether, the Rice
Creek UAS ecosystem provides a high quality autopilot system that
ephasizes high reliability and simple code.  It offers many advanced
capabilities at a very inexpensive price point.

# Features

* MPU9250 via spi or i2c.
* MPU9250 DMP, interrupt generatation, scaling
* Gyro zeroing (calibration) automatically on startup if unit is still enough.
* UBLOX8 support
* 15-State EKF (kalman filter), 2 variants: (1) ins/gns, (2) ins/gns/mag
* SBUS input (direct) with support for 16 channels.
* BME280/BMP280 support
* Eeprom support for saving/loading config as well as assigning a serial #.
* 8 channel PWM output support
* Onboard 3-axis stability (simple dampening) system
* "Smart reciever" capability.  Handles major mixing and modes on the
  airplane side allowing flight with a 'dumb' radio.
* i2c airdata system (BFS, mRobotics, 3dr)
* Full 2-way serial communication with any host computer enabling a 
  "big processor/little processor" architecture.  Hard real time tasks run on
  the "little" processor.  High level functions run on the big processor (like
  EKF, PID's, navigation, logging, communication, mission, etc.)

# Flight Testing

* This system originated in the late 2000's and has been evolving as
  DIY haredware has improved.  The APM2 version of the firmware flew
  at least as early as 2012.  There was an APM1 version prior to that
  and a Xbow MNAV version even earlier.  The PJRC-teensy version of
  this system has been flying since February 2018.

# What's new in 2020?

* 2020 brought us the Teensy 4.0 (yeah!) with crazy fast CPU speeds
  and more memory.  This allows us to imagine doing more of the flight
  critical work right on the 'little' processor, leaving the 'big'
  processor for the tasks that can run at slightly slower rates.

* I have pushed through quite a few code architecture and
  simplification changes.  The goal is always to make the structure
  lighter weight when possible.

* I have added a powerful matrix based inceptor->effector mixing
  system.  The mixes can be setup logically by function, or by
  defining the matrix directly.

* I have added support for an on-board strapdown error calibration
  matrix, on-board accelerometer calibration and onboard magenetometer
  calibration.

* I have added two variants of the UMN AEM ins/gns kalman filter.

  1. A 15-state ins/gps only filter (gyros, accels, gps) that performs
     extremely well for fixed wing aircraft.

  2. A 15-state ins/gps/mag filter that supports low dynamic vehicles
     such as quad copters and rovers.  As with any magnetomter based
     attitude determination system, it is critical to have well
     calibrated mags for good performance.  I have an offline "self"
     calibration system that I am considering adapting for
     onboard/automatic calibration.

* A subset of the covariances are reported to the host: (1) the
  maximum of the 3 position errors, (2) the max of the 3 velocity
  errors, (3) the max of the 3 attitude errors.  These are statistical
  estimates, but can be useful for monitoring the health of the ekf
  solution.

# What is next?

* I would like to investigate running inner loop PID control onboard
  the teensy (offloaded from the host.)  This would lead to an
  extremely tight inner main loop: sense -> state estimator -> pid
  control -> effector output.  The higher level navigation would
  remain on the host computer as well as other functions like logging
  and communication with the ground station.


# BUILD INSTRUCTIONS

(rev1 ... sketchy note phase ...)

* git pull ardupilot source tree
* git pull rc-fmu source tree (to an independent location)
* cd .../ardupilot
* ln -s .../path/to/rc-fmu .
* edit ardupilot/wscript to include rc-fmu as a program name

* edit …/ardupilot/libraries/AP_Vehicle/AP_Vehicle_Type.h to #define
  APM_BUILD_Subdir with a new value (I picked 101 to stay out of the
  way of possible future expansion.) But most likely these changes
  will live only on my own hard drive. Note: the symbol you define
  uses the subdir name, not the common name you picked.

* I had to do extra steps to get Eigen3 to work inside the AP_HAL /
  ChibiOS build environment.  Please see docs/README-Eigen3.md for the
  required changes and work arounds.

* ./waf configure --board XYZ
* ./waf rc-fmu --upload

