package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo calibration", group="NONONONONONONONOOOOO")
public class ServoCalibration extends LinearOpMode {

    Servo servo;
    @Override
    public void runOpMode(){
        servo = hardwareMap.servo.get("servo");
        double position = 0.5;
        boolean a_now;
        boolean b_now;
        boolean x_now;
        boolean y_now;
        boolean a_then = false;
        boolean b_then = false;
        boolean x_then = false;
        boolean y_then = false;

        waitForStart();
        while(opModeIsActive()){
            a_now = gamepad1.a;
            b_now = gamepad1.b;
            x_now = gamepad1.x;
            y_now = gamepad1.y;
            if (a_now && !a_then){
                position += 0.1;
            } else if (b_now && !b_then){
                position -= 0.1;
            } else if (x_now && !x_then){
                position += 0.01;
            } else if (y_now && !y_then){
                position -= 0.001;
            }
            a_then = a_now;
            b_then = b_now;
            x_then = x_now;
            y_then = y_now;
            servo.setPosition(position);
            telemetry.addData("Position: ", position);
            telemetry.update();
        }

    }
}
