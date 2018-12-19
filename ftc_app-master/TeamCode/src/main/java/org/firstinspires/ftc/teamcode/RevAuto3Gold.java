
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;

@Autonomous(name = "Rev Auto 3 Gold", group="default")
//@Disabled
public class RevAuto3Gold extends LinearOpMode {

    Selectron s;

    @Override
    public void runOpMode() {

        s = new Selectron(this);

        waitForStart();

        s.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        s.r.reset();



        s.timeAngleEncoderDrive(-90, 60, 0.4);
        s.timeAngleEncoderDrive(90, 2, 0.4);

        s.timeAngleEncoderDrive(180, 42, 0.4);

        s.dropper.setPosition(s.DROPPER_OUT);
        s.snooze(200);

        s.timeAngleEncoderDrive(0, 78, 0.4);
    }
}