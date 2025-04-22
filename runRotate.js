var __interpretation_started_timestamp__;
var pi = 3.141592653589793;

var ML = brick.motor(M4);
var MR = brick.motor(M3);

var EL = brick.encoder(E4);
var ER = brick.encoder(E3);

// 2D constants
var cpr = 360;
var d = 5.6;
var b = 15.4;

// Real robot constants
//var cpr = 574;
//var d = 8.5;
//var b = 19.5;

// Number of full rotations
var rotations = 0;
// Yaw from previous measurement
var yawOld = 0;
// Absolute angle updated by timer based on gyroscope
var angle = 0;

function createTimer(ms, func) {
    t = script.timer(ms);
    t.timeout.connect(func);
    return t
}

var cmToCycles = function(dist) {
    return dist * cpr / (pi * d);
}

var angleToCycles = function(alpha) {
    return cmToCycles(b * alpha / 2);
}

var setPowerAll = function(power) {
    MR.setPower(power);
    ML.setPower(power);
}

var run = function(v, s) {
    var startEncoder = ER.read();
    var startEncoder2 = EL.read();
    var initDiff = startEncoder2 - startEncoder;
    var c = 1;
    var err = 0;
    var dist = cmToCycles(s)
    setPowerAll(v);
    while (Math.abs(ER.read() - startEncoder) < dist) {
        err = EL.read() - ER.read() - initDiff;
        MR.setPower(v + c * err);
        ML.setPower(v - c * err);
        script.wait(10);
    }
    setPowerAll(0);
}

var runGyro = function(v, s) {
    var startEncoder = ER.read();
    var initAngle = angle;
    var c = 1;
    var err = 0;
    var dist = cmToCycles(s)
    setPowerAll(v);
    // var i = 0;
    while (Math.abs(ER.read() - startEncoder) < dist) {
        err = angle - initAngle;
        // if (i % 15 == 0) { print(err) }
        MR.setPower(v + c * err);
        ML.setPower(v - c * err);
        script.wait(20);
    }
    setPowerAll(0);
}

function runHybrid(v, s) {
    var startEncoder = ER.read();
    var startEncoder2 = EL.read();
    var initAngle = angle;
    var initDiff = startEncoder2 - startEncoder;
    var c = 1;
    var a = 0.5;
    var err = 0;
    var dist = cmToCycles(s)
    setPowerAll(v);
    while (Math.abs(ER.read() - startEncoder) < dist) {
        err1 = angle - initAngle;
        err2 = EL.read() - ER.read() - initDiff;
        err = a * err1 + (1 - a) * err2;
        MR.setPower(v + c * err);
        ML.setPower(v - c * err);
        script.wait(20);
    }
    setPowerAll(0);
}

var rotate = function(v, alpha) {
    if (alpha < 0) {
        v *= -1;
        alpha *= -1;
    }
    var startEncoder = ER.read()
    var dist = angleToCycles(alpha);
    MR.setPower(v);
    ML.setPower(-v);
    while (Math.abs(ER.read() - startEncoder) < dist) {
        script.wait(10);
    }
    setPowerAll(0);
}

var rotateGyro = function(v, alpha) {
    if (alpha < 0) {
        v *= -1;
        alpha *= -1;
    }
    var initAngle = angle;
    MR.setPower(v);
    ML.setPower(-v);
    while (Math.abs(angle - initAngle) < alpha) {
        script.wait(10);
    }
    setPowerAll(0);
}

var gyroSetup = function() {
    brick.gyroscope().setCalibrationValues([107, -30, 37, 208, 89, 4083])
}

function updateAngle() {
    var yaw = brick.gyroscope().read()[6] / 1000;
    var diff = yaw - yawOld;
    yawOld = yaw;
    rotations += -Math.round(diff / 320);
    angle = yaw + rotations * 360;
}

function initAngle() {
    yawOld = brick.gyroscope().read()[6] / 1000;
    script.wait(100);
    yawOld = brick.gyroscope().read()[6] / 1000;
    angle = yawOld;
}

var main = function() {
    __interpretation_started_timestamp__ = Date.now();
    initAngle();

    var timer = createTimer(200, function() {
        updateAngle();
    })

    rotate(40, -pi);
    runHybrid(50, 100);

    timer.stop();


    return;
}

