# History

The gimbal code here started as a direct copy of the Ardupilot
AP_Mount system.

The code has been modified (simple changes to class names and file
names) to avoid potential confusion with the system library versions.

The code has been further modified to extract the AP parameter system
(essentially the purpose of the front end) and replace it with the
property system.

Initially there is only interest in the SToRM32 driver (via mavlink)
so that is all the code that has been migrated here and subjected to
these modifications.

More heavy modifications to support gremsy gimbal setup and optimal
modes for our pointing use case.

## Gremsy Notes

* Full gimbal must be powered on (not just the mount interface) to
  talk to it and read/write anything.

* Pay careful attention to Gremsy cable pinout ... this is not a
  standard drone-code cable pinout and tx/rx need to be crossed over.

* Got the gSDK working with FTDI cable jumpered to COM2 on Gremsy
  (Gnd, Tx, Rx)

* Must send mavlink HB at 1hz or gimbal will revert to default
  pointing mode (using pwm inputs.)


## Pointing notes

Put the gimbal in lock mode (holds position specified) otherwise will
'follow' mount orientation smoothly.

Default 'lock' mode commands send pitch/roll angles to 'world'
coordinates so you are depending on the gimbal's imu/ekf for pitch and
roll angles which may or may not be that great.

roll and pitch angles we send to gimbal are in ned frame as estimated
by the gimbal.  We can compute ned azimuth and send that directly (we
don't care about our own body orienation here.)

Still need to work out what is going on with relative vs. absolute yaw
angles reported by gimbal.