# Rice Creek FMU

This is an Ardupilot/ChibiOS based flight controller firmware that
serves as the heart of a Rice Creek UAV autopilot.

Note: the purpose of this project is to prototype and demonstrate
various autopilot design ideas.  Occasionally I like to break the rules
or make up my own rules and in some places, this project shows how
that might look.  Also it is a priority to maintain this system in a
state that is robust and flyable so that these ideas can be proven in
actual flight test.

This README has only been partially updated for the Ardupilot/ChibiOS
port, so some of it is likely wrong or outdated.  The AP/ChibiOS
version of the code is at an advanced checkpoint state ... nearly
working, but still a work in progress and not yet flight tested as of
June 2021.

## Demonstrated design ideas

* Property tree: a hierarchical data structure composed of nested
  arrays & dicts.
  * Implements a cooperative publish/subscribe system.
  * Simplifies and centralizes the data flow between modules.
  * Integrates cleanly with json (reading and writing.)
  * Brings some conveniences of scripted languages to C++.  Flexibility
    of run time data creation, reference by ascii name, type agnostic.
  * Built on top of rapidjson which provides all the low level tree
    building and accessor functionality.
  * Eliminates (somewhat) the tangled web of interconnected #include
    dependencies that grow organically with many substantial C++ projects.
  * Eliminates most initialization order dependencies.
  * Supports a single interface (via the shared property tree) to
    integrated script modules called from anywhere in the code.
    
* Thread-less design: grand loop structure. (excepting the background
  service and driver threads that are part of the underlying AP_HAL
  libraries.)

* [depricating] Big processor / little processor architecture moves
  important functionality to the host (big) computer.  Simultaneously
  enables a much simpler and lighter weight "little" processor.
  Dividing the workload between two systems leads to two simpler apps
  versus one single very complicated monolithic app.  However, I am
  actually moving more nad more functionality to the 'little'
  processor because it is not so little on modern flight controllers.

* [deprecating] Extensive use of python and scripted tasks on the big
  processor where python is supported.

* Accel calibration procedure that generates an affine matrix
  (encapsulates rotation, bias, and scale errors in a single step.)

* [todo?] Dynamic accelerometer temperature calibration. Trusts the EKF's
  accel bias estimates and fits a model of those estimates vs
  temperature over time.

* [definitely not done, but todo?] Dynamic compass calibration.
  Trusts the EKF's attitude estimate and world magnetic model to fit a
  compass calibration model incrementally over time.

* Nested json configuration system with an extension to allow one
  config file to include sub-configuration files.  (Compared to a flat
  array of config parameters, configuration can include strings and
  arrays as well.)

* Simple message header generation (json definition file) for
  communicating with host computer and logging.  Supports C++ and
  python compatible messages.  It is like a mavlink-lite, and very
  lite.

* University of Minnesota, Aerospace Engineering and Mechanics, UAV
  lab 15-state EKF (and magnetometer version.)  Designed to run
  continuously and ubiquitously.

* [in process] On board HIL sim physics engine reduces the number of
  parts to assemble in your house of cards to do HIL testings.

## Some major bullet point todo list items:

- need to test drive ekf15_mag with some sort of preliminary mag
  calibration.
  
- pid's
- switches / modes
  - ap/thr-safe based on switch config or hard coded?
  - a 3-pos switch in binary mode ... what happens in that ill defined
    middle state?
  
- tecs
- temp and mag calibration based on EKF when it is in a high confidence state
- affine_from_points() verbosity
- direct sd card logging
- decide how best to detect and choose when multiple imu and compass
  are available.
- Is there a way we can activate console type messages over the
  telemetry port for convenience when the system is installed and
  harder to access the usb console port?

* (x) figure out/test airdata connections (external port on mro gps - ms5525)
  - (x) for now just hacked the default sensor in AP_Airdata.cpp
* (x) test servo outputs (seems to be working?)
  - (x) throttle scaling wrong (not-symmetrical)
* (x) Breadboard with pixracer + beaglebone (for lab testing)
  - (x) external power / 5v from where?  Needs to power both pixracer and
    beaglebone @ 5v
  - (x) telemetry: beaglebone usb
  - (x) custom uart cable from pixracer <-> beaglebone
* (x) fix gps message
* (x) rc-flight side driver, messaging, and 2 byte pkt_len serial_link version.
* (x) serial_link packet communication: (x) read len (x)write len (x)
  checksum (x) payload new/realloc
* (x) port ekf15 heap allocation changes over to ekf15_mag
* (x) look through config messages and see what needs to be changed for
  the new system. (new rc-flight driver to match this updated so we
  can continue to support the goldy3 variant separately as long as we
  have that hardware in play.)
* (x) move accel calibration code over and test.
  - (x) double check strapdown rotation matrix extracted from affine is correct
  - (x) strap down (for rotating gyros & mags) vs affine (for rotation,
    scale, and bias of accels.)
  - (x) implement fit quality metrics
  - (x) save to sd card
* (x) figure out the wscript magic to allow me to organize my code in
  subdirectories
* (x) sort out how I really want to organize classes vs. singletons
  vs. global instances vs. namespace vs. cross-wired interdependencies
  between everything.  I have some messy mixes of these due to deep
  dive, sprint development dynamics.
  * notice that statically allocating big things avoids potentially
    blowing up the stack space on some super cramped boards.
* (x) look carefully at what is in setup_board.h and where this is
  #included all about
* (x) figure out led's (AP_Notify)
* (x) the_gps and the_imu are not great names. (_mgr is also not great,
  but I've used it before and I hate it less ...)
* (x) do a round of file renaming, or figure out how to orgainize code in
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

## Features

* Any AP supported IMU.
* Any AP supported GPS
* Any AP supported RC in/out system.
* Any AP supported air data sensor.
* AP supporte EEPROM (storage).
* 15-State EKF (kalman filter), 2 variants: (1) ins/gns, (2) ins/gns/mag
* Onboard 3-axis stability (simple dampening) system
* "Smart reciever" capability.  Handles major mixing and modes on the
  airplane side allowing flight with a 'dumb' radio.
* Full 2-way serial communication with any host computer enabling a 
  "big processor/little processor" architecture.  Hard real time tasks run on
  the "little" processor.  High level functions run on the big processor (like
  EKF, PID's, navigation, logging, communication, mission, etc.)
* Single grand loop architecture -- focus on simplicity over features.

## Flight Testing

* This system originated in the late 2000's and has been evolving as
  DIY haredware has improved.  The APM2 version of the firmware flew
  at least as early as 2012.  There was an APM1 version prior to that
  and a Xbow MNAV version even earlier.  The PJRC-teensy version of
  this system has been flying since February 2018.  This new AP_HAL /
  ChibiOS version is interoduced in 2021.

## What is new in 2021?

* I am working on porting the entire fmu application to the Ardupilot
  / AP_HAL /ChibiOS ecosystem.  I am using their 'capabilities' (low
  level drivers and abstractions) to implement the functionality of
  the FMU code (versus doing this all in the arduino ecosystem.)

  * Advantages: Optimized drivers and backend.  Access to a nice set
    of flight controller boards with sensors and power management and
    standard connectors.  Access to modules that support things like
    can bus, and sd card storage (which we currently don't support
    now.)
    
  * Disadvantages: no teensy-3.6 support so this deprecates our
    in-house hardware (or we need to support two devel trees for a
    while.)  Not everything is done the same or available exactly as
    it is in arduino ... so for example, serial/uart IO is only
    serviced at 50hz which prevents us from doing high rate
    communication with an external host computer.

## What is new in 2020?

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

* Simple property tree implementation for config flexibility?

## BUILD INSTRUCTIONS

(rev1 ... sketchy note phase ...)

* git pull ardupilot source tree
* git pull rc-fmu-ap source tree (to an independent location)
* cd .../ardupilot
* ln -s .../path/to/rc-fmu-ap .
* edit ardupilot/wscript to include rc-fmu-ap as a program name

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

