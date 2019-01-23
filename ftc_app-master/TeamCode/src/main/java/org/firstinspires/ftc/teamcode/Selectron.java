package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class Selectron {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor lift;
    DcMotor spin;
    DcMotor intake;

    Servo dropper;
    Servo colorS;

    LinearOpMode op;

    double DROPPER_IN = 0.7;
    double DROPPER_OUT = 0.3;

    BNO055IMU imu;

    ElapsedTime r;

    public Selectron(LinearOpMode opMode){



        op = opMode;
        dropper = op.hardwareMap.servo.get("drop");
        colorS = op.hardwareMap.servo.get("colorS");


        lift = op.hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(BRAKE);

        intake = op.hardwareMap.get(DcMotor.class, "intake");
        spin = op.hardwareMap.get(DcMotor.class, "spin");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        bl = op.hardwareMap.dcMotor.get("bL");
        br = op.hardwareMap.dcMotor.get("bR");
        fr = op.hardwareMap.dcMotor.get("fL");
        fl = op.hardwareMap.dcMotor.get("fR");

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);


        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        r = new ElapsedTime();
    }

    public double findHeading(){
        return imu.getAngularOrientation().firstAngle;
    }

    public void snooze(double milliseconds){
        double startTime = r.milliseconds();
        while (r.milliseconds() < startTime + milliseconds && op.opModeIsActive()){
        }
    }

    public void timeAngleDrive(int angleDegrees, double milliseconds){
        timeAnglePowerDrive(angleDegrees, milliseconds, 1.0);
    }

    public void timeAnglePowerDrive(int angleDegrees, double milliseconds, double power){
        double flbrPower = power * Math.cos(Math.PI * (angleDegrees + 45)/180.0);
        double frblPower = power * Math.sin(Math.PI * (angleDegrees + 45)/180.0);
        fl.setPower(flbrPower);
        fr.setPower(frblPower);
        bl.setPower(frblPower);
        br.setPower(flbrPower);
        snooze(milliseconds);
        stopMotors();
        snooze(200);
    }

    // 0 is the forwards
    public void timeAngleEncoderDrive(int angleDegrees, double inches, double power){
        double encoderCounts = 1120*inches/(4*Math.PI);
        int flbrDistance = (int)(encoderCounts * Math.cos(Math.PI * (angleDegrees + 45)/180.0));
        int frblDistance = (int)(encoderCounts * Math.sin(Math.PI * (angleDegrees + 45)/180.0));
        double flbrPower = power * (Math.cos(Math.PI * (angleDegrees + 45)/180.0));
        double frblPower = power * (Math.sin(Math.PI * (angleDegrees + 45)/180.0));
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
        snooze(100);
    }
    public void stopMotors(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void setEncoders(DcMotor.RunMode runMode){
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }
}