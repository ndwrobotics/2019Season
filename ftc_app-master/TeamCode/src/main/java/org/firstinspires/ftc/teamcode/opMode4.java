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

        s.bl.setPower(1);
        s.snooze(1000);
        s.bl.setPower(0);
        s.br.setPower(1);
        s.snooze(1000);
        s.br.setPower(0);
        s.fl.setPower(1);
        s.snooze(1000);
        s.fl.setPower(0);
        s.fr.setPower(1);
        s.snooze(1000);
        s.fr.setPower(0);



        while (opModeIsActive()) {

            double y = -gamepad1.right_stick_y;
            double x = gamepad1.right_stick_x;
            int desiredAngle = (int) (180*Math.atan2(y, x));
            double magnitude = Math.sqrt(Math.pow(y,2) + Math.pow(x,2));

        }
    }
}
