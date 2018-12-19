/*
[][][] [][][] [][]  [][][]
[]     []  [] [] [] [][]
[][][] [][][] [][]  [][][]
* */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

@Autonomous(name = "Rev Auto 3", group="default")
//@Disabled
public class RevAuto4BlueGold extends LinearOpMode {

    ElapsedTime tm = new ElapsedTime();

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    Servo dropper;
    Servo colorServo;

    int heading;
    int zeroHeading;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    double DROPPER_IN = 0.04;
    double DROPPER_OUT = 0.8;

    double COLOR_IN = 0.5;
    double COLOR_OUT = 0.5;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;


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


        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "color");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        dropper = hardwareMap.servo.get("dropper");
        colorServo = hardwareMap.servo.get("color");

        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");

        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl.setMode(RUN_TO_POSITION);
        br.setMode(RUN_TO_POSITION);
        fl.setMode(RUN_TO_POSITION);
        fr.setMode(RUN_TO_POSITION);




        zeroHeading = (int)findHeading();


        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        tm.reset();

        boolean found = false;

        colorServo.setPosition(COLOR_IN);

        timeAngleEncoderDrive(180, 24, 0.4);
        motorPowerAngles(-45, 0.25);
        while(sensorDistance.getDistance(DistanceUnit.INCH) > 3.0){}
        stopMotors();
        motorPowerAngles(-135, 0.25);
        while(sensorDistance.getDistance(DistanceUnit.INCH) > 1.5){}
        stopMotors();
        if (sensorColor.blue() > 0.2 && sensorColor.green() > 0.2){
            found = true;
            timeAngleEncoderDrive(-135, 6, 0.4);
            timeAngleEncoderDrive(45, 8, 0.4);
        }

        timeAngleEncoderDrive(-45, 20, 0.4);

        timeAngleEncoderDrive(-90, 60, 0.4);
        timeAngleEncoderDrive(90, 2, 0.4);

        timeAngleEncoderDrive(180, 42, 0.4);

        dropper.setPosition(DROPPER_OUT);
        s(200);

        timeAngleEncoderDrive(0, 75, 0.4);
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
        motorPowerAngles(angleDegrees, power);
        s(milliseconds);
        stopMotors();
        s(200);
    }

    // 0 is the forwards
    public void timeAngleEncoderDrive(int angleDegrees, double inches, double power){
        double encoderCounts = 1120*inches/(4*Math.PI);
        int flbrDistance = (int)(encoderCounts * Math.cos(Math.PI * (angleDegrees + 45)/180.0));
        int frblDistance = (int)(encoderCounts * Math.sin(Math.PI * (angleDegrees + 45)/180.0));
        fl.setTargetPosition(fl.getCurrentPosition() + flbrDistance);
        br.setTargetPosition(br.getCurrentPosition() + flbrDistance);
        fr.setTargetPosition(fr.getCurrentPosition() + frblDistance);
        bl.setTargetPosition(bl.getCurrentPosition() + frblDistance);
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        while(opModeIsActive() && (fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy())){}
        stopMotors();
        s(200);
    }

    public void motorPowerAngles(int angleDegrees, double power){
        double flbrPower = power * Math.cos(Math.PI * (angleDegrees + 45)/180.0);
        double frblPower = power * Math.sin(Math.PI * (angleDegrees + 45)/180.0);
        fl.setPower(flbrPower);
        fr.setPower(frblPower);
        bl.setPower(frblPower);
        br.setPower(flbrPower);
    }

    public void stopMotors(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}