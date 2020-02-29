package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous

public class ServoClampTest extends LinearOpMode{

    private Servo clamperL;
    private Servo testServo;
    private Servo clamperR;


    private void connectToHardwareMap(){
        //connecting program names to hardware map names on robot controller phone
        clamperL = hardwareMap.get(Servo.class, "clamperL");
        // testServo = hardwareMap.get(Servo.class, "crazyTestServo");
        clamperR = hardwareMap.get(Servo.class, "clamperR");

        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        connectToHardwareMap();

        telemetry.addData("ServoA Pos", clamperL.getPosition());
        telemetry.addData("ServoB Pos", clamperR.getPosition());
        telemetry.update();

        //clamperA.setPosition(0.0);
        //clamperB.setPosition(0.0);

        waitForStart();

        telemetry.addData("ServoA Pos", clamperL.getPosition());
        telemetry.addData("ServoB Pos", clamperR.getPosition());
        telemetry.update();
        while (opModeIsActive()) {
            /* if (clamperA.getPosition() == 0.5) */
            clamperL.setPosition(0.0);
            clamperR.setPosition(0.5);
        }
    }
}