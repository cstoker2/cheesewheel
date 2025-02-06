/*

2/6 figured the glitch - sick fwd meant it went 90 right, stick back meant went 90 left, so had to make a seprate decoupled phase calc for the leds.

1/14 tweaked stickangle to start at 12oclock instead of 3oclock...
1/13/25  testing shows more stability with klman filter and only using z accel data. kalman Q of about 1 seems most stable.

rotation is CW, when translating forward, the heading seems to bounce about 90 deg CCW. 
crappy loose battery managment circuit made for dropouts on the teensy power bus.


*/