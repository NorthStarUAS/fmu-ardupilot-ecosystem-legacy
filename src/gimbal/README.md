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