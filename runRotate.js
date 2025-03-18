var __interpretation_started_timestamp__;
var pi = 3.141592653589793;

// 2D constants
var cpr = 360;
var d = 5.6;
var b = 15.4;

// Real robot constants
//var cpr = 574;
//var d = 8.5;
//var b = 19.5;

var cmToCycles = function(dist) {
	return dist * cpr / (pi * d);
}

var angleToCycles = function(alpha) {
	return cmToCycles(b * alpha / 2);
}

var setPowerAll = function(power) {
	brick.motor(M3).setPower(power);
	brick.motor(M4).setPower(power);
}

var run = function(v, s) {
	var startEncoder = brick.encoder(E3).read();
	var startEncoder2 = brick.encoder(E4).read();
	var initDiff = startEncoder2 - startEncoder;
	var c = 1;
	var err = 0;
	var dist = cmToCycles(s)
	setPowerAll(v);
	while (Math.abs(brick.encoder(E3).read() - startEncoder) < dist) {
		err = brick.encoder(E4).read() - brick.encoder(E3).read() - initDiff;
		brick.motor(M3).setPower(v + c * err);
		brick.motor(M4).setPower(v - c * err);
		script.wait(10);
	}
	setPowerAll(0);
}

var rotate = function(v, alpha) {
	if (alpha < 0) {
		v *= -1;
		alpha *= -1;
	}
	var startEncoder = brick.encoder(E3).read()
	var dist = angleToCycles(alpha);
	brick.motor(M3).setPower(v);
	brick.motor(M4).setPower(-v);
	while (Math.abs(brick.encoder(E3).read() - startEncoder) < dist) {
		script.wait(10);
	}
	setPowerAll(0);
}

var main = function()
{
	__interpretation_started_timestamp__ = Date.now();
	
	run(60, 100);
	rotate(40, pi / 2);
	
	return;
}

