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

@Autonomous(name = "Rev Auto P2", group="default")
//@Disabled
public class RevAutoGoldP2 extends LinearOpMode {

    ElapsedTime tm = new ElapsedTime();

    Selectron s;

    @Override
    public void runOpMode() {

        s = new Selectron(this);

        waitForStart();

        s.timeAngleEncoderDrive(45, 60, 1.0);
        s.dropper.setPosition(s.DROPPER_OUT);
        s.snooze(200);
        s.timeAngleEncoderDrive(-135, 60, 1.0);
        s.timeAngleEncoderDrive(-135, 6, 0.4);

        tm.reset();

    }

}