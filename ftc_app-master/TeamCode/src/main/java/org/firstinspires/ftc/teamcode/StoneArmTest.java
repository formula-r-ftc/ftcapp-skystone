package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class  StoneArmTest extends LinearOpMode {

    private Servo stoneArmR;
    private Servo stoneArmL;
    private Servo stoneArmClampR;
    private Servo stoneArmClampL;

    @Override
    public void runOpMode() {

        stoneArmR = hardwareMap.get(Servo.class, "stoneArmR");
        stoneArmL = hardwareMap.get(Servo.class, "stoneArmL");
        stoneArmClampR = hardwareMap.get(Servo.class, "stoneArmClampR");
        stoneArmClampL = hardwareMap.get(Servo.class, "stoneArmClampL");

    double tgtPower = 0;
    while (opModeIsActive()) {

        if (gamepad1.x) {
            stoneArmR.setPosition(0.0);
        } else if (gamepad1.y) {
            stoneArmClampR.setPosition(0.0);
        } else if (gamepad1.y) {
            stoneArmL.setPosition(0.0);
        } else if (gamepad1.x) {
            stoneArmClampL.setPosition(0.0);
        }

        telemetry.addData("Servo Position", stoneArmR.getPosition());
        telemetry.addData("Servo Position", stoneArmL.getPosition());
        telemetry.addData("Servo Position", stoneArmClampR.getPosition());
        telemetry.addData("Servo Position", stoneArmClampL.getPosition());
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Status", "Running");
        telemetry.update();
        }
    }
}
