/*
[][][] [][][] [][]  [][][]
[]     []  [] [] [] [][]
[][][] [][][] [][]  [][][]
* */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "Rev Auto 1", group="default")
//@Disabled
public class RevAuto1 extends LinearOpMode {

    ElapsedTime tm = new ElapsedTime();

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    Servo dropper;

    int heading;
    int zeroHeading;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    double DROPPER_IN = 0.04;
    double DROPPER_OUT = 0.8;


    @Override
    public void runOpMode() {

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        dropper = hardwareMap.servo.get("dropper");

        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);


        zeroHeading = (int)findHeading();

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        tm.reset();

        heading = (int)findHeading();


        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));


        double desiredHeading = Math.atan2(y, x);

        int desiredHeadingDegrees = (int) (desiredHeading * 180.0/Math.PI);


        telemetry.addData("desiredHeadingDegrees: ", desiredHeadingDegrees);
        telemetry.addData("heading: ", heading);



        int error = desiredHeadingDegrees - heading;
        error = error % 360;
        telemetry.addData("error: ", error);


        int angle = zeroHeading + 180 - error;

        timeAngleDrive(90, 1500);
        timeAngleDrive(0, 1000);
        dropper.setPosition(DROPPER_OUT);

        timeAngleDrive(180, 2000);
        timeAnglePowerDrive(180, 2000, 0.6);
    }

    public double findHeading(){
        return imu.getAngularOrientation().firstAngle;
    }

    public void s(double milliseconds){
        double startTime = tm.milliseconds();
        while (tm.milliseconds() < startTime + milliseconds && opModeIsActive()){
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
        s(milliseconds);
        stopMotors();
        s(200);
    }
    public void stopMotors(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}