/*
[][][] [][][] [][]  [][][]
[]     []  [] [] [] [][]
[][][] [][][] [][]  [][][]
* */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "headless omni op", group="default")
//@Disabled
public class HeadlessOmniOp extends OpMode {

    ElapsedTime tm = new ElapsedTime();

    DcMotor left;
    DcMotor right;
    DcMotor front;
    DcMotor back;

    ModernRoboticsI2cGyro gyro;

    int heading;
    int zeroHeading;
    double GYRO_COMPENSATION_FACTOR = 83.0/90.0;

    @Override
    public void init() {

        back = hardwareMap.dcMotor.get("back");
        front = hardwareMap.dcMotor.get("front");
        right = hardwareMap.dcMotor.get("right");
        left = hardwareMap.dcMotor.get("left");

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        front.setDirection(DcMotor.Direction.REVERSE);
        back.setDirection(DcMotor.Direction.FORWARD);

        gyro.calibrate();
        while (gyro.isCalibrating()){

        }
        zeroHeading = (int)findHeading();


    }

    @Override
    public void start () {

        tm.reset();

    }

    @Override
    public void loop() {
        heading = (int)findHeading();


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;

        double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));


        double desiredHeading = Math.atan2(y, x);

        int desiredHeadingDegrees = (int) (desiredHeading * 180.0/Math.PI);


        telemetry.addData("desiredHeadingDegrees: ", desiredHeadingDegrees);
        telemetry.addData("heading: ", heading);



        int error = heading - desiredHeadingDegrees;
        error = error % 360;
        telemetry.addData("error: ", error);


        int angle = zeroHeading + 180 - error;
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





        left.setPower(0.5*(lrPower + turnPower));
        right.setPower(0.5*(lrPower - turnPower));
        front.setPower(0.5*(udPower + turnPower));
        back.setPower(0.5*(udPower - turnPower));


        if (gamepad1.a){
            zeroHeading = (int)findHeading();
        }
    }

    public double findHeading(){
        return gyro.getIntegratedZValue()/GYRO_COMPENSATION_FACTOR;
    }

}