package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="driv 3", group="")
//@Disabled
public class opMode3 extends LinearOpMode {

    Selectron s;

    private ElapsedTime runtime = new ElapsedTime();

    boolean down = false;

    int zeroHeading;
    int heading;
    Orientation angles;

    @Override
    public void runOpMode() {

        s = new Selectron(this);


        waitForStart();

        while (opModeIsActive()) {

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


            if (gamepad1.a) {
                zeroHeading = (int) s.findHeading();
            }


            if(gamepad2.right_stick_y>0.5){
                s.spin.setPower(1000);
            } else if (gamepad2.right_stick_y<-0.5) {
                s.spin.setPower(-1000);
            } else {
                s.spin.setPower(0);
            }

            if(gamepad2.b){
                s.dropper.setPosition(0);
            } else if (gamepad2.a) {
                s.dropper.setPosition(1);
            }

            if((gamepad1.left_bumper&&Math.abs(s.lift.getCurrentPosition())<7000)||gamepad1.y){
                s.lift.setPower(-0.5);
            } else if ((gamepad1.left_trigger>=0.5&&Math.abs(s.lift.getCurrentPosition())>50)||gamepad1.a) {
                s.lift.setPower(0.5);
            } else { s.lift.setPower(0); }telemetry.addData("lift", s.lift.getCurrentPosition());telemetry.update();

            runtime.reset();

            if(gamepad2.dpad_up&&down == true){
                while (opModeIsActive()&&s.intake.getCurrentPosition()<0) {
                    s.intake.setPower(0.5);
                    s.intake.setTargetPosition(0);
                    down = false;
                }
            } else if (gamepad2.dpad_down&&down == false) {
                while (opModeIsActive()&&s.intake.getCurrentPosition()>-1100) {
                    s.intake.setPower(-0.5);
                    s.intake.setTargetPosition(-1100);
                    down = true;
                }
            } else {
                s.intake.setPower(0);
            }

            telemetry.addData("pos", s.intake.getCurrentPosition());

            telemetry.update();

        }
    }
}
