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

# Notes

* Full gimbal must be powered on (not just the mount interface) to
  talk to it and read/write anything.

* Burns battery crazy fast

* Got the gSDK working with FTDI cable jumpered to COM2 on Gremsy
  (Gnd, Tx, Rx)

* See if I can understand what linux sdk is doing that I'm not on the
  pixhawk4

# Pointing notes:

Put the gimbal in lock mode (holds position specified)

roll and pitch angles we send to gimbal are in ned frame as estimated
by the gimbal.  We can compute ned azimuth and send that directly (we
don't care about our own body orienation here.)

Still need to work out what is going on with relative vs. absolute yaw
angles reported by gimbal.