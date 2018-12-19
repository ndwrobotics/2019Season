/*
[][][] [][][] [][]  [][][]
[]     []  [] [] [] [][]
[][][] [][][] [][]  [][][]
* */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "headless omni op Rev", group="default")
//@Disabled
public class HeadlessOmniOpRev extends OpMode {

    ElapsedTime tm = new ElapsedTime();

    DcMotor fl;
    DcMotor br;
    DcMotor fr;
    DcMotor bl;

    int heading;
    int zeroHeading;

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;




    @Override
    public void init() {

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



        bl = hardwareMap.dcMotor.get("bl");
        fr = hardwareMap.dcMotor.get("fr");
        br = hardwareMap.dcMotor.get("br");
        fl = hardwareMap.dcMotor.get("fl");

        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);



        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        zeroHeading = (int)findHeading();


    }

    @Override
    public void start () {

        tm.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }

    @Override
    public void loop() {
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


        int angle = zeroHeading + 225 - error;
        double lrPower;
        double udPower;
        if (magnitude > 0.02) {
            lrPower = Math.cos(angle * Math.PI / 180.0) * magnitude;
            udPower = Math.sin(angle * Math.PI / 180.0) * magnitude;
            telemetry.addData("lrPower: ", lrPower);
            telemetry.addData("udPower: ", udPower);
        } else {
            lrPower = 0;
            udPower = 0;
        }

        double turnPower = gamepad1.right_stick_x;

        double lPower = lrPower + turnPower;
        double rPower = lrPower - turnPower;
        double uPower = udPower + turnPower;
        double dPower = udPower - turnPower;

        double max = Math.abs(Math.max(Math.max(lPower, rPower), Math.max(uPower, dPower)));

        if(max > 1.0){
            lPower /= max;
            rPower /= max;
            uPower /= max;
            dPower /= max;
        }

        fl.setPower(lPower);
        br.setPower(rPower);
        fr.setPower(uPower);
        bl.setPower(dPower);


        if (gamepad1.a){
            zeroHeading = (int)findHeading();
        }
    }

    public double findHeading(){
        return imu.getAngularOrientation().firstAngle;
    }

}