//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU.AccelUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Selectron2 {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor lift;
    Servo dropper;
    Servo colorS;
    LinearOpMode op;
    double DROPPER_IN = 0.04D;
    double DROPPER_OUT = 0.8D;
    double COLOR_IN = 0.1D;
    double COLOR_OUT = 0.6D;
    BNO055IMU imu;
    ElapsedTime r;

    public Selectron2(LinearOpMode opMode) {
        op = opMode;
        dropper = (Servo)op.hardwareMap.servo.get("drop");
        colorS = (Servo)op.hardwareMap.servo.get("colorS");
        lift = (DcMotor)op.hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        Parameters parameters = new Parameters();
        parameters.angleUnit = AngleUnit.DEGREES;
        parameters.accelUnit = AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = (BNO055IMU)op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        bl = (DcMotor)op.hardwareMap.dcMotor.get("bL");
        br = (DcMotor)op.hardwareMap.dcMotor.get("bR");
        fr = (DcMotor)op.hardwareMap.dcMotor.get("fL");
        fl = (DcMotor)op.hardwareMap.dcMotor.get("fR");
        bl.setDirection(Direction.REVERSE);
        br.setDirection(Direction.FORWARD);
        fl.setDirection(Direction.REVERSE);
        fr.setDirection(Direction.FORWARD);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setMode(RunMode.RUN_TO_POSITION);
        br.setMode(RunMode.RUN_TO_POSITION);
        fl.setMode(RunMode.RUN_TO_POSITION);
        fr.setMode(RunMode.RUN_TO_POSITION);
        r = new ElapsedTime();
    }

    public double findHeading() {
        return (double)imu.getAngularOrientation().firstAngle;
    }

    public void snooze(double milliseconds) {
        double startTime = r.milliseconds();

        while(r.milliseconds() < startTime + milliseconds && op.opModeIsActive()) {
            ;
        }

    }

    public void timeAngleDrive(int angleDegrees, double milliseconds) {
        timeAnglePowerDrive(angleDegrees, milliseconds, 1.0D);
    }

    public void timeAnglePowerDrive(int angleDegrees, double milliseconds, double power) {
        double flbrPower = power * Math.cos(3.141592653589793 * (double)(angleDegrees + 45) / 180.0);
        double frblPower = power * Math.sin(3.141592653589793 * (double)(angleDegrees + 45) / 180.0);
        fl.setPower(flbrPower);
        fr.setPower(frblPower);
        bl.setPower(frblPower);
        br.setPower(flbrPower);
        snooze(milliseconds);
        stopMotors();
        snooze(200.0D);
    }

    public void timeAngleEncoderDrive(int angleDegrees, double inches, double power) {
        double encoderCounts = 1120.0D * inches / 12.566370614359172D;
        int flbrDistance = (int)(encoderCounts * Math.cos(3.141592653589793D * (double)(angleDegrees + 45) / 180.0D));
        int frblDistance = (int)(encoderCounts * Math.sin(3.141592653589793D * (double)(angleDegrees + 45) / 180.0D));
        double flbrPower = power * Math.cos(3.141592653589793D * (double)(angleDegrees + 45) / 180.0D);
        double frblPower = power * Math.sin(3.141592653589793D * (double)(angleDegrees + 45) / 180.0D);
        if(Math.abs(frblPower) < 0.05){
            frblPower = 0.05;
        }
        if(Math.abs(flbrPower) < 0.05){
            flbrPower = 0.05;
        }
        fl.setTargetPosition(fl.getCurrentPosition() + flbrDistance);
        br.setTargetPosition(br.getCurrentPosition() + flbrDistance);
        fr.setTargetPosition(fr.getCurrentPosition() + frblDistance);
        bl.setTargetPosition(bl.getCurrentPosition() + frblDistance);
        fl.setPower(flbrPower);
        fr.setPower(frblPower);
        bl.setPower(frblPower);
        br.setPower(flbrPower);

        while(op.opModeIsActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())){}

        stopMotors();
        snooze(100.0D);
    }

    public void stopMotors() {
        fl.setPower(0.0D);
        fr.setPower(0.0D);
        bl.setPower(0.0D);
        br.setPower(0.0D);
    }

    public void setEncoders(RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
        while(op.opModeIsActive() && ((fl.getMode() != runMode) || (fr.getMode() != runMode) || (bl.getMode() != runMode) || (br.getMode() != runMode))){}
        snooze(10);
    }
}
