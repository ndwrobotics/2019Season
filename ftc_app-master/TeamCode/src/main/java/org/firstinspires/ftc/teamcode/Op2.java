/*
[][][] [][][] [][]  [][][]
[]     []  [] [] [] [][]
[][][] [][][] [][]  [][][]
* */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "Op 2", group="default")
//@Disabled
public class Op2 extends LinearOpMode {

    ElapsedTime tm = new ElapsedTime();


    int heading;
    int zeroHeading;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;



    Selectron s;


    @Override
    public void runOpMode() {

        s = new Selectron(this);
        s.setEncoders(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        zeroHeading = (int) s.findHeading();
        tm.reset();

        s.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while(opModeIsActive()) {
            heading = (int) s.findHeading();


            angles = s.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));


            double desiredHeading = Math.atan2(y, x);

            int desiredHeadingDegrees = (int) (desiredHeading * 180.0 / Math.PI);


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

            double max = Math.max(Math.max(Math.abs(lPower), Math.abs(rPower)), Math.max(Math.abs(uPower), Math.abs(dPower)));

            if (max > 1.0) {
                lPower /= max;
                rPower /= max;
                uPower /= max;
                dPower /= max;
            }

            s.fl.setPower(lPower);
            s.br.setPower(rPower);
            s.fr.setPower(uPower);
            s.bl.setPower(dPower);


            if (gamepad1.left_trigger > 0.5) {
                zeroHeading = (int) s.findHeading();
            }


            if (gamepad1.x) {
                s.robotLift.setPower(0.6);
            } else if (gamepad1.b){
                s.robotLift.setPower(-0.6);
            } else {
                s.robotLift.setPower(0);
            }
            telemetry.addData("Robot lift position: ", s.robotLift.getCurrentPosition());
            telemetry.update();
        }
    }

}