package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class ArmTest extends LinearOpMode{
   private CRServo servoArm;
   private ElapsedTime runtime = new ElapsedTime();
   static final double FORWARD_SPEED = -1;
    
    @Override
        public void runOpMode() {
        servoArm = hardwareMap.get(CRServo.class, "servoArm");
        
        waitForStart();
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1.5){
                servoArm.setPower(-FORWARD_SPEED);
                telemetry.addData("armPower", servoArm.getPower());
                telemetry.addData("armPort", servoArm.getPortNumber());
                telemetry.update();
                
            }
        }
}
