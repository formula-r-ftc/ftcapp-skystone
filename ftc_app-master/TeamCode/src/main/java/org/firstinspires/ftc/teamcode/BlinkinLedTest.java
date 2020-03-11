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
    private RevBlinkinLedDriver.BlinkinPattern SlowIntakeOn;
    private RevBlinkinLedDriver.BlinkinPattern OutakeOn;
    private RevBlinkinLedDriver.BlinkinPattern IntakeNOutakeOff;

    private ElapsedTime t1 = new ElapsedTime();

    private boolean toggleIntakeBoolean = false;
    private boolean toggleOutakeBoolean = false;

    private void IntakeToggle(){
        if (gamepad2.left_trigger > 0.5 && t1.seconds() > 0.5 ){
            t1.reset();
            toggleIntakeBoolean = true;
            toggleOutakeBoolean = false;
        }
    }

    private void IntakeButtons(){
        if (gamepad1.a && t1.seconds() > 0.5 ){
            toggleIntakeBoolean = true;
            toggleOutakeBoolean = false;
        }
        else if (gamepad1.b){
            toggleIntakeBoolean = false;
            toggleOutakeBoolean = true;
        }
        else if (gamepad1.x){
            toggleIntakeBoolean = false;
            toggleOutakeBoolean = false;
        }
    }

    private void intakeControl(){
        if (!gamepad1.right_bumper) {
            if (toggleIntakeBoolean) {
                blinkin.setPattern(IntakeOn);
                IntakeL.setPower(0.5);
                IntakeR.setPower(-0.5);
            } else if (toggleOutakeBoolean) {
                blinkin.setPattern(OutakeOn);
                IntakeL.setPower(-0.5);
                IntakeR.setPower(0.5);
            } else if (!toggleIntakeBoolean && !toggleOutakeBoolean) {
                blinkin.setPattern(IntakeNOutakeOff);
                IntakeL.setPower(0.0);
                IntakeR.setPower(0.0);
            }
        }
        else{
            if (toggleIntakeBoolean) {
                blinkin.setPattern(SlowIntakeOn);
                IntakeL.setPower(0.3);
                IntakeR.setPower(-0.3);
            } else if (toggleOutakeBoolean) {
                blinkin.setPattern(OutakeOn);
                IntakeL.setPower(-0.5);
                IntakeR.setPower(0.5);
            } else if (!toggleIntakeBoolean && !toggleOutakeBoolean) {
                blinkin.setPattern(IntakeNOutakeOff);
                IntakeL.setPower(0.0);
                IntakeR.setPower(0.0);
            }
        }
    }

    private void OutakeToggle(){
        if (gamepad2.right_trigger > 0.5 && t1.seconds() > 0.5 ){
            t1.reset();
            toggleOutakeBoolean=true;
            toggleIntakeBoolean=false;
        }
    }

    @Override
    public void runOpMode() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinky");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeNOutakeOff = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        IntakeOn = RevBlinkinLedDriver.BlinkinPattern.RED;
        OutakeOn = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        SlowIntakeOn = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
        waitForStart();
        while (opModeIsActive()) {

            IntakeToggle();
            OutakeToggle();
            IntakeButtons();
            intakeControl();
        }
    }
}

