package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="driv 4", group="")
//@Disabled
public class opMode4 extends LinearOpMode {

    Selectron s;

    private ElapsedTime runtime = new ElapsedTime();

    boolean down = false;

    int zeroHeading;
    int heading;
    Orientation angles;

    @Override
    public void runOpMode() {

        s = new Selectron(this);


        s.setEncoders(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        /*s.bl.setPower(1);
        telemetry.addLine("bl");
        telemetry.update();
        s.snooze(1000);
        s.bl.setPower(0);
        s.br.setPower(1);
        telemetry.addLine("br");
        telemetry.update();
        s.snooze(1000);
        s.br.setPower(0);
        s.fl.setPower(1);
        telemetry.addLine("fl");
        telemetry.update();
        s.snooze(1000);
        s.fl.setPower(0);
        s.fr.setPower(1);
        telemetry.addLine("fr");
        telemetry.update();
        s.snooze(1000);
        s.fr.setPower(0);*/

        // all positive is forwards.

        while (opModeIsActive()) {

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;
            double desiredAngle = Math.atan2(y, x);
            double magnitude = Math.sqrt(Math.pow(y,2) + Math.pow(x,2));

            if (magnitude > 0.05){
                angleTurnDrive(magnitude, desiredAngle, gamepad1.left_stick_x);
            } else {

                angleDrive(0, 0);
            }

        }
    }
    public void angleDrive(double power, double angle){
        /*
        front right and back left should be positive for 0-90, negative 180-270
        back right and front left should be positive for 0, negative for 90-180, positive for 270
        +45
        front right and back left: sin
        back right and front left: cos
         */
        double actualAngle = angle - Math.PI/4.0;
        double frblPower = power * Math.sin(actualAngle);
        double flbrPower = power * Math.cos(actualAngle);

        s.fr.setPower(frblPower);
        s.fl.setPower(flbrPower);
        s.bl.setPower(frblPower);
        s.br.setPower(flbrPower);
    }
    public void angleTurnDrive(double power, double angle, double turnAmount){
        /*
        front right and back left should be positive for 0-90, negative 180-270
        back right and front left should be positive for 0, negative for 90-180, positive for 270
        +45
        front right and back left: sin
        back right and front left: cos
         */
        /*
        positive number on the gamepad x-axis should turn right
        all positive makes the robot go forward
        so add the x-value to the front left and back left
        and subtract the x-value from the front right and back right
         */
        double actualAngle = angle - Math.PI/4.0;
        double frblPower = power * Math.sin(actualAngle);
        double flbrPower = power * Math.cos(actualAngle);

        double flPower = flbrPower + turnAmount;
        double frPower = frblPower - turnAmount;
        double brPower = flbrPower - turnAmount;
        double blPower = frblPower + turnAmount;

        s.fr.setPower(frPower);
        s.fl.setPower(flPower);
        s.bl.setPower(blPower);
        s.br.setPower(brPower);
    }
}
