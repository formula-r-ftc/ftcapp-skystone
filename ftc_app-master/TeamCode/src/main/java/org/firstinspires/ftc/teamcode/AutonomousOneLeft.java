package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutonomousOneLeft extends LinearOpMode {
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private ElapsedTime runtime = new ElapsedTime();

    //Setting variables for speeds
    static final double FORWARD_SPEED = 0.5;

    @Override
    public void runOpMode() {
        //connecting program names to hardware map names on robot controller phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        //Entering while loop
        while (opModeIsActive() && runtime.seconds() < 2) {
            runtime.reset();

            RFMotor.setPower(-FORWARD_SPEED);
            LFMotor.setPower(FORWARD_SPEED);
            RBMotor.setPower(-FORWARD_SPEED);
            LBMotor.setPower(FORWARD_SPEED);
            break;
        }
        while (opModeIsActive() && runtime.seconds() < 2) {
            runtime.reset();

            RFMotor.setPower(FORWARD_SPEED);
            LFMotor.setPower(FORWARD_SPEED);
            RBMotor.setPower(-FORWARD_SPEED);
            LBMotor.setPower(-FORWARD_SPEED);
            break;
        }
    }
}