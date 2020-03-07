package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RightStoneArmTest extends LinearOpMode {
    private Servo stoneArmR;
    private Servo stoneArmClampR;
    private Servo stoneArmL;
    private Servo stoneArmClampL;




    @Override
    public void runOpMode() {

        stoneArmR = hardwareMap.get(Servo.class, "stoneArmR");
        stoneArmClampR = hardwareMap.get(Servo.class, "stoneArmClampR");
        stoneArmL = hardwareMap.get(Servo.class, "stoneArmL");
        stoneArmClampL = hardwareMap.get(Servo.class, "stoneArmClampL");

        waitForStart();
        while (opModeIsActive()) {

           if(gamepad1.a) {
           stoneArmR.setPosition(0.0);
           } else {
           stoneArmR.setPosition(0.5);
           }

           if(gamepad1.b) {
               stoneArmClampR.setPosition(0.0);
           } else {
           stoneArmClampR.setPosition(0.5);
           }

           if(gamepad1.x) {
               stoneArmClampL.setPosition(0.0);
           } else {
               stoneArmClampL.setPosition(0.5);
           }

           if(gamepad1.y) {
               stoneArmL.setPosition(0.0);
           }else {
               stoneArmL.setPosition(0.5);
           }

            telemetry.addData("Servo Position", stoneArmR.getPosition());
            telemetry.addData("Servo Position", stoneArmClampR.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
