package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class StoneArmTest extends LinearOpMode {

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

    waitForStart();
    while (opModeIsActive()) {

        if (gamepad1.x) {
            stoneArmR.setPosition(1.0);
        } else if (gamepad1.y) {
            stoneArmClampR.setPosition(1.0);
        } else if (gamepad1.a) {
            stoneArmL.setPosition(1.0);
        } else if (gamepad1.b) {
            stoneArmClampL.setPosition(1.0);
        }

        telemetry.addData("Servo Position", stoneArmR.getPosition());
        telemetry.addData("Servo Position", stoneArmL.getPosition());
        telemetry.addData("Servo Position", stoneArmClampR.getPosition());
        telemetry.addData("Servo Position", stoneArmClampL.getPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();
        }
    }
}
