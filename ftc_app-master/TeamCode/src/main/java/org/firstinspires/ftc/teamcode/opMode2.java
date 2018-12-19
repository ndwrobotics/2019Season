package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@TeleOp(name="driv", group="")
//@Disabled
public class opMode2 extends LinearOpMode {

    Selectron s;

    private ElapsedTime runtime = new ElapsedTime();

    boolean down = false;

    @Override
    public void runOpMode() {

        s = new Selectron(this);


        waitForStart();

        while (opModeIsActive()) {


            double yr = -gamepad1.right_stick_y;
            double xr = gamepad1.right_stick_x;
            double xl = -gamepad1.left_stick_x;

            if (gamepad1.x){
                yr*=0.5;
                xr*=0.5;
                xl*=0.5;
            }

            if (Math.abs(yr)>0) {

                s.bl.setPower(yr);
                s.fl.setPower(yr);
                s.br.setPower(yr);
                s.fr.setPower(yr);

            } else if (Math.abs(xr)>0) {

                s.bl.setPower(-xr);
                s.fl.setPower(-xr);
                s.br.setPower(xr);
                s.fr.setPower(xr);

            } else if ( Math.abs(xl)>0 ) {
                s.bl.setPower(-xl*0.5);
                s.fl.setPower(xl*0.5);
                s.br.setPower(xl*0.5);
                s.fr.setPower(-xl*0.5);
            } else {
                s.bl.setPower(0);
                s.fl.setPower(0);
                s.br.setPower(0);
                s.fr.setPower(0);
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
