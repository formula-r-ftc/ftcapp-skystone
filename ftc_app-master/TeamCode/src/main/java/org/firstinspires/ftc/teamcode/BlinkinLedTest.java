package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class BlinkinLedTest extends LinearOpMode {
    private DcMotor IntakeR;
    private DcMotor IntakeL;
    private RevBlinkinLedDriver blinkin;
    private RevBlinkinLedDriver.BlinkinPattern IntakeOn;
    private RevBlinkinLedDriver.BlinkinPattern OutakeOn;
    private RevBlinkinLedDriver.BlinkinPattern IntakeNOutakeOff;

    private ElapsedTime t1 = new ElapsedTime();

    private boolean toggleIntakeBoolean = false;
    private boolean toggleOutakeBoolean = false;

    private void IntakeToggle(){
        if (gamepad1.left_stick_button && t1.seconds() > 0.5 || gamepad2.left_trigger > 0.5 && t1.seconds() > 0.5 ){
            t1.reset();
            if (!toggleIntakeBoolean){
                IntakeL.setPower(0.5);
                IntakeR.setPower(-0.5);
                toggleIntakeBoolean=true;
            } else if (toggleIntakeBoolean){
                IntakeL.setPower(0);
                IntakeR.setPower(0);
                toggleIntakeBoolean=false;
            }
        }
    }

    private void OutakeToggle(){
        if (gamepad1.right_stick_button && t1.seconds() > 0.5 || gamepad2.right_trigger > 0.5 && t1.seconds() > 0.5 ){
            t1.reset();
            if (!toggleOutakeBoolean){
                IntakeL.setPower(-0.5);
                IntakeR.setPower(0.5);
                toggleOutakeBoolean=true;
            } else if (toggleOutakeBoolean){
                IntakeL.setPower(0);
                IntakeR.setPower(0);
                toggleOutakeBoolean=false;
            }
        }
    }
    @Override
    public void runOpMode() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinky");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeNOutakeOff = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        IntakeOn = RevBlinkinLedDriver.BlinkinPattern.RED;
        OutakeOn = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        waitForStart();
        while (opModeIsActive()) {

            IntakeToggle();
            OutakeToggle();

            if (!toggleIntakeBoolean && !toggleOutakeBoolean){
                blinkin.setPattern(IntakeNOutakeOff);
            }
            else if (toggleIntakeBoolean){
                blinkin.setPattern(IntakeOn);
            }
            else if (toggleOutakeBoolean) {
                blinkin.setPattern(OutakeOn);
            }
        }
    }
}

