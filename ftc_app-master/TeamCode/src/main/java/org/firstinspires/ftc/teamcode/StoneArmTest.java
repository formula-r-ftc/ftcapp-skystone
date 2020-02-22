package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class StoneArmTest extends LinearOpMode {

    private Servo stoneArmA;
    private Servo stoneArmB;
    private Servo stoneArmClampA;
    private Servo stoneArmClampB;

    @Override
    public void runOpMode() {

        stoneArmA = hardwareMap.get(Servo.class, "stoneArmA");
        stoneArmB = hardwareMap.get(Servo.class, "stoneArmB");
        stoneArmClampA = hardwareMap.get(Servo.class, "stoneArmClampA");
        stoneArmClampB = hardwareMap.get(Servo.class, "stoneArmClampB");

    double tgtPower = 0;
    while (opModeIsActive()) {

        if (gamepad1.y) {
            stoneArmA.setPosition(0.0);
        } else if (gamepad1.b) {
            stoneArmClampA.setPosition(0.0);
        } else if (gamepad1.x) {
            stoneArmB.setPosition(0.0);
        } else if (gamepad1.a) {
            stoneArmClampB.setPosition(0.0);
        }

        telemetry.addData("Servo Position", stoneArmA.getPosition());
        telemetry.addData("Servo Position", stoneArmB.getPosition());
        telemetry.addData("Servo Position", stoneArmClampA.getPosition());
        telemetry.addData("Servo Position", stoneArmClampB.getPosition());
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Status", "Running");
        telemetry.update();
        }
    }
}
