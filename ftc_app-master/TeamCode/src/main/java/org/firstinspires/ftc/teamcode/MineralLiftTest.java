package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="mineral lift test", group="")
//@Disabled
public class MineralLiftTest extends LinearOpMode {

    ElapsedTime r = new ElapsedTime();
    DcMotor mineralLift;
    DcMotor intakeLift;
    int intakePos = 0;
    int mineralPos = 0;

    boolean aNow;
    boolean aThen = false;
    boolean yNow;
    boolean yThen = false;
    boolean uNow;
    boolean uThen = false;
    boolean dNow;
    boolean dThen = false;
    @Override
    public void runOpMode() {
        mineralLift = hardwareMap.dcMotor.get("mineral");
        intakeLift = hardwareMap.dcMotor.get("intake");
        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mineralLift.setPower(0.6);
        intakeLift.setPower(0.6);

        waitForStart();


        while(opModeIsActive()){
            aNow = gamepad1.a;
            yNow = gamepad1.y;
            uNow = gamepad1.dpad_up;
            dNow = gamepad1.dpad_down;

            if (aNow && !aThen){
                intakePos -= 200;
            }

            if (yNow && !yThen){
                intakePos += 200;
            }

            if(dNow && !dThen){
                mineralPos -= 500;
            }
            if (uNow && !uThen){
                mineralPos += 500;
            }
            if(gamepad1.x){
                mineralLift.setPower(0);
                intakeLift.setPower(0);
            }

            mineralLift.setTargetPosition(mineralPos);
            intakeLift.setTargetPosition(intakePos);

            telemetry.addData("Mineral pos: ", mineralPos);
            telemetry.addData("Intake pos: ", intakePos);
            telemetry.update();

            aThen = aNow;
            yThen = yNow;
            dThen = dNow;
            uThen = uNow;
        }

    }
}
