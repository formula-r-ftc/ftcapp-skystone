package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class TwoBarTelemetry extends LinearOpMode {
    private Servo BlockPusher;

    @Override
    public void runOpMode() {
        BlockPusher = hardwareMap.get(Servo.class, "stoneArmR");
        waitForStart();
        BlockPusher.setPosition(0.0);
        telemetry.addData("Servo Pos", BlockPusher.getPosition());
        telemetry.update();
    }
}