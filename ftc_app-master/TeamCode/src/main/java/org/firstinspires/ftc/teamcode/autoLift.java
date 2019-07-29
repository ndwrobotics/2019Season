//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(
        name = "autoLift",
        group = ""
)
public class autoLift extends LinearOpMode {
    Selectron2 s;

    public void runOpMode() {
        ElapsedTime r = new ElapsedTime();
        s = new Selectron2(this);
        DistanceSensor distanceSensor = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "sensorcol");
        s.lift.setPower(0.0);
        telemetry.addData("", s.dropper.getPosition());
        telemetry.update();
        waitForStart();
        s.lift.setPower(-0.75);

        while(Math.abs(s.lift.getCurrentPosition()) < 7500 && opModeIsActive()) {
            s.snooze(1.0);
            telemetry.addData("lift", s.lift.getCurrentPosition());
            telemetry.update();
        }

        s.lift.setPower(0.0);
        sleep(100);
        s.dropper.setPosition(1.0);
        s.colorS.setPosition(s.COLOR_OUT-0.05);
        s.timeAngleEncoderDrive(180, 4.0, 0.5);
        s.lift.setPower(0.75);

        while(Math.abs(s.lift.getCurrentPosition()) > 3000 && opModeIsActive()) {
            s.snooze(1.0);
            telemetry.addData("lift", s.lift.getCurrentPosition());
            telemetry.update();
        }

        s.lift.setPower(0);
        s.timeAngleEncoderDrive(0, 5.0, 0.5);
        s.timeAngleEncoderDrive(45, 26.0, 0.5);
        s.bl.setMode(RunMode.RUN_USING_ENCODER);
        s.fl.setMode(RunMode.RUN_USING_ENCODER);
        s.br.setMode(RunMode.RUN_USING_ENCODER);
        s.fr.setMode(RunMode.RUN_USING_ENCODER);
        s.bl.setPower(-0.25);
        s.fl.setPower(-0.25);
        s.br.setPower(-0.25);
        s.fr.setPower(-0.25);
        r.reset();

        while(!isYellow() && opModeIsActive() && r.seconds() < 8.0) {
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (distanceSensor.getDistance(DistanceUnit.INCH) < 15.0) {
                s.bl.setPower(0.0);
                s.fl.setPower(0.0);
                s.br.setPower(0.0);
                s.fr.setPower(0.0);
                s.snooze(50.0);

                while(distanceSensor.getDistance(DistanceUnit.INCH) > 6.0 && opModeIsActive()) {
                    s.timeAngleEncoderDrive(90, 0.5, 0.4);
                    telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }

                if (!isYellow()) {
                    s.setEncoders(RunMode.RUN_TO_POSITION);
                    s.timeAngleEncoderDrive(-180, 6.0, 0.5);
                    telemetry.addLine("No yellow");
                    s.setEncoders(RunMode.RUN_USING_ENCODER);
                    telemetry.update();
                    s.bl.setPower(-0.25);
                    s.fl.setPower(-0.25);
                    s.br.setPower(-0.25);
                    s.fr.setPower(-0.25);
                }
            }
        }

        s.bl.setMode(RunMode.RUN_TO_POSITION);
        s.fl.setMode(RunMode.RUN_TO_POSITION);
        s.br.setMode(RunMode.RUN_TO_POSITION);
        s.fr.setMode(RunMode.RUN_TO_POSITION);
        s.timeAngleEncoderDrive(90, 10.0, 1.0);
        s.timeAngleEncoderDrive(-90, 10.0, 1.0);
        s.bl.setMode(RunMode.RUN_USING_ENCODER);
        s.fl.setMode(RunMode.RUN_USING_ENCODER);
        s.br.setMode(RunMode.RUN_USING_ENCODER);
        s.fr.setMode(RunMode.RUN_USING_ENCODER);
        s.bl.setPower(-0.5);
        s.fl.setPower(-0.5);
        s.br.setPower(-0.5);
        s.fr.setPower(-0.5);

        while(!(distanceSensor.getDistance(DistanceUnit.INCH) <= 9.0) && opModeIsActive()) {
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            sleep(1);
        }

        s.bl.setPower(0.0);
        s.fl.setPower(0.0);
        s.br.setPower(0.0);
        s.fr.setPower(0.0);
        s.bl.setMode(RunMode.RUN_TO_POSITION);
        s.fl.setMode(RunMode.RUN_TO_POSITION);
        s.br.setMode(RunMode.RUN_TO_POSITION);
        s.fr.setMode(RunMode.RUN_TO_POSITION);

        s.colorS.setPosition(s.COLOR_IN);

        s.timeAngleEncoderDrive(45, 12, 1.0);
        s.timeAngleEncoderDrive(-90, 4, 1.0);
        s.timeAngleEncoderDrive(0, 60, 1.0);
        s.dropper.setPosition(s.DROPPER_OUT);
        s.snooze(200);
        s.timeAngleEncoderDrive(-180, 60, 1.0);
        s.timeAngleEncoderDrive(-180, 6, 0.4);

    }

    boolean isYellow() {
        ColorSensor sensorColor = (ColorSensor)hardwareMap.get(ColorSensor.class, "sensorcol");
        float[] hsvValues = new float[]{0, 0, 0};
        double SCALE_FACTOR = 255.0D;
        Color.RGBToHSV((int)((double)sensorColor.red() * 255.0), (int)((double)sensorColor.green() * 255.0D), (int)((double)sensorColor.blue() * 255.0), hsvValues);
        if (hsvValues[0] < 40.0 && hsvValues[0] > 20.0 && (double)hsvValues[1] > 0.4 && (double)hsvValues[1] < 0.7) {
            telemetry.addData("yellow", hsvValues[0]);
            telemetry.update();
            return true;
        } else {
            telemetry.addData("yellow", hsvValues[0]);
            telemetry.update();
            return false;
        }
    }

}













































