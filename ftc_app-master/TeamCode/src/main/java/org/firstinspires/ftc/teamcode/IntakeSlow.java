package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class IntakeSlow extends LinearOpMode {

    private DcMotor IntakeL;
    private DcMotor IntakeR;

    private ElapsedTime t1 = new ElapsedTime();

    boolean toggleIntakeBoolean = false;
    boolean toggleOutakeBoolean = false;

    private void moveIntake() {
        if (gamepad1.left_bumper && gamepad1.right_trigger < 0.5 && t1.seconds() > 0.5) {

            if (!toggleOutakeBoolean) {
                IntakeL.setPower(-0.3);
                IntakeR.setPower(0.3);
                toggleOutakeBoolean = true;
            } else if (toggleOutakeBoolean) {
                IntakeL.setPower(0);
                IntakeR.setPower(0);
                toggleOutakeBoolean = false;
            } else {

                if (!toggleOutakeBoolean) {
                    IntakeL.setPower(-0.5);
                    IntakeR.setPower(0.5);
                    toggleOutakeBoolean = true;
                } else if (toggleOutakeBoolean) {
                    IntakeL.setPower(0);
                    IntakeR.setPower(0);
                    toggleOutakeBoolean = false;
                }

            }

        }
    }
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()){
            moveIntake();
        }
    }

}