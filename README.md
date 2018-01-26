# casper

A small robot for Halloween.

The robot is based on Actobotics Sprout.  It has 2 drive wheels and a stabilizer.  The motors include gearboxes,
and the rear shafts of the motors extend out from the motors.

The robot is controlled by a Raspberry Pi.

Disks with a 50% interrupter pattern are attached to the rear motor shafts, and optical beam-break sensors are set in a
quadrature configuation.  The outputs are fed to a Raspberry Pi.

A kernel driver decodes the quad inputs and returns counts for both motors when read.
