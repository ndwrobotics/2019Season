/*
[][][] [][][] [][]  [][][]
[]     []  [] [] [] [][]
[][][] [][][] [][]  [][][]
* */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "Rev Auto Silver 3", group="default")
//@Disabled
public class RevAuto3Silver extends LinearOpMode {

    Selectron s;

    @Override
    public void runOpMode() {

        s = new Selectron(this);

        waitForStart();

        s.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        s.r.reset();



        s.timeAngleEncoderDrive(90, 60, 0.4);
        s.timeAngleEncoderDrive(-90, 2, 0.4);

        s.timeAngleEncoderDrive(180, 60, 0.4);

        s.dropper.setPosition(s.DROPPER_OUT);
        s.snooze(200);

        s.timeAngleEncoderDrive(0, 72, 0.4);
    }
}