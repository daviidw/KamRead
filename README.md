# KamRead
Readout from kamstrup 602 heat meter

Based on arduino code from Torsten Martinsen
http://wiki.hal9k.dk/projects/kamstrup

Originally based on PyKamstrup by Erik Jensens
https://github.com/bsdphk/PyKamstrup

------------
For software serial a modified version of AltSoftSerial by Paul Stoffregen was used
Original code:
https://github.com/PaulStoffregen/AltSoftSerial
https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html

Changes made to allow readout of when all tx data have been sent

------------
KamRead

Converted for non blocking operation.
Result data in decimal int (x10 for one decimal)
Error code in last integer, so result data array must have an extra integer for this.  
------------
