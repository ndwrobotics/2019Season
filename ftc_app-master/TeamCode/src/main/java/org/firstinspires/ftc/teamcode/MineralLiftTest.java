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
    boolean bNow;
    boolean bThen = false;
    boolean slow = false;
    @Override
    public void runOpMode() {
        mineralLift = hardwareMap.dcMotor.get("mineral");
        intakeLift = hardwareMap.dcMotor.get("intake");

        mineralLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mineralLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        mineralLift.setPower(0.6);
        intakeLift.setPower(0.6);


        waitForStart();


        while(opModeIsActive()) {
            uNow = gamepad1.dpad_up;
            dNow = gamepad1.dpad_down;
            /*
            aNow = gamepad1.a;
            yNow = gamepad1.y;
            uNow = gamepad1.dpad_up;
            dNow = gamepad1.dpad_down;
            bNow = gamepad1.b;
            if(bNow && !bThen){
                slow = !slow;
            }
            if(slow){
                if (aNow && !aThen){
                    intakePos -= 20;
                }

                if (yNow && !yThen){
                    intakePos += 20;
                }

                if(dNow && !dThen){
                    mineralPos -= 50;
                }
                if (uNow && !uThen){
                    mineralPos += 50;
                }
            } else {
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
            bThen = bNow;
            */

            if(gamepad1.a){
                intakeLift.setTargetPosition(-2600);
            }
            if(gamepad1.y){
                intakeLift.setTargetPosition(-1200);
            }
            if(gamepad1.x){
                intakeLift.setTargetPosition(20);
            }


            if(dNow && !dThen){
                mineralPos -= 500;
            }
            if (uNow && !uThen){
                mineralPos += 500;
            }
            dThen = dNow;
            uThen = uNow;
            telemetry.addData("Mineral pos: ", mineralPos);
            telemetry.addData("Intake pos: ", intakePos);
            telemetry.update();
        }
    }
}
