package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;

@Autonomous

public class ServoClampTest extends LinearOpMode{

    private Servo clamperA;
    private Servo testServo;
    private Servo clamperB;


    private void connectToHardwareMap(){
        //connecting program names to hardware map names on robot controller phone
        clamperA = hardwareMap.get(Servo.class, "clamperA");
        // testServo = hardwareMap.get(Servo.class, "crazyTestServo");
        clamperB = hardwareMap.get(Servo.class, "clamperB");

        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        connectToHardwareMap();

        telemetry.addData("ServoA Pos", clamperA.getPosition());
        telemetry.addData("ServoB Pos", clamperB.getPosition());
        telemetry.update();

        //clamperA.setPosition(0.0);
        //clamperB.setPosition(0.0);

        waitForStart();

        telemetry.addData("ServoA Pos", clamperA.getPosition());
        telemetry.addData("ServoB Pos", clamperB.getPosition());
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("ServoB Pos", clamperB.getPosition());
            telemetry.update();


            /* if (clamperA.getPosition() == 0.5) */
            clamperA.setPosition(0.0);
            clamperB.setPosition(0.5);
        }
    }
}