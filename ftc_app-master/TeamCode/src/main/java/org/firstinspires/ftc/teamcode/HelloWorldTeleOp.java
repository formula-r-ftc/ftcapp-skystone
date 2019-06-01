package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class HelloWorldTeleOp extends LinearOpMode {
    //naming units referenced in program;
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private CRServo Arm;

    @Override
    public void runOpMode() {
        //connecting program names to hardware map names on robot controller phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        Arm = hardwareMap.get(CRServo.class, "Arm");

        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Entering while loop
        while (opModeIsActive()) {

            //Sending status to drivers station
            telemetry.addData("Status", "Running");
            telemetry.update();

            //Connecting the power of the RFMotor to the right stick y axis of controller 1
            double RFtgtPower = 0;
            RFtgtPower = gamepad1.right_stick_y;
            RFMotor.setPower(RFtgtPower);
            telemetry.addData(" RF Target Power", RFtgtPower);
            telemetry.addData(" RF Motor Power", RFMotor.getPower());
            telemetry.update();

            //Connecting the power of the RBMotor to the left stick y axis of controller 1
            double RBtgtPower = 0;
            RBtgtPower = gamepad1.left_stick_y;
            RBMotor.setPower(RBtgtPower);
            telemetry.addData("RB Target Power", RBtgtPower);
            telemetry.addData("RB Motor Power", RBMotor.getPower());
            telemetry.update();

            //Connecting the power of the LFMotor to the right stick y axis of controller2
            double LFtgtPower = 0;
            LFtgtPower = gamepad2.right_stick_y;
            LFMotor.setPower(LFtgtPower);
            telemetry.addData("LF Target Power", LFtgtPower);
            telemetry.addData("LF Motor Power", LFMotor.getPower());
            telemetry.update();

            //Connecting the power of the LBMotor to the left stick y axis of controller 2
            double LBtgtPower = 0;
            LBtgtPower = gamepad2.left_stick_y;
            LBMotor.setPower(LBtgtPower);
            telemetry.addData("LB Target Power", LBtgtPower);
            telemetry.addData("LB Motor Power", LBMotor.getPower());
            telemetry.update();

            //Connecting the positive power of the Arm to the right trigger of controller 1
            double ArmtgtPower = 0;
            ArmtgtPower = gamepad1.right_trigger;
            Arm.setPower(ArmtgtPower);
            telemetry.addData("Arm Target Power", ArmtgtPower);
            telemetry.addData("Arm Motor Power", Arm.getPower());

            //Connecting the negative power of the Arm to the left trigger of controller 1
            double NArmtgtPower = 0;
            NArmtgtPower = -gamepad1.left_trigger;
            Arm.setPower(NArmtgtPower);
            telemetry.addData("Negative Arm Target Power", NArmtgtPower);
            telemetry.addData("Arm Motor Power", Arm.getPower());
        }
    }

}
