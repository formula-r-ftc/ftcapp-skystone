package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class FormulaRTeleOpFinal extends LinearOpMode {

    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private DcMotor slidePlacer;
    private DcMotor LinearSlide;
    private CRServo servoArm;
    private CRServo grabbingServo;
    //private Servo turner;
    private DcMotor twoBarLift;
    private Servo clamper;
    static final double one = 746.66666666;
    private double targetPosTwoBarLift = 0;
    private double targetPosLinearSlide = 0;

    private void runIntake() {
        double intakePower = 0;
        intakePower = this.gamepad2.left_trigger;
        if (intakePower >= 0.1 && intakePower <= 0.5) {
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        } else if (intakePower >= 0.5) {
            IntakeL.setPower(-0.65);
            IntakeR.setPower(0.65);
        }
        telemetry.addData("Intake Motor Power", IntakeR.getPower());
        telemetry.addData("Intake Motor Power", IntakeL.getPower());
    }

    private void runOutake() {
        double outakePower = 0;
        outakePower = this.gamepad2.right_trigger;
        if (outakePower >= 0.1 && outakePower <= 0.5) {
            IntakeL.setPower(0);
            IntakeR.setPower(0);
        } else if (outakePower >= 0.5) {
            IntakeL.setPower(1);
            IntakeR.setPower(-1);
        }
        telemetry.addData("OutakePower", outakePower);
        telemetry.addData("Outake Motor Power", IntakeR.getPower());
        telemetry.addData("Outake Motor Power", IntakeL.getPower());
    }

    private void runLinearSlide() {
        double LinearSlidePower = 0;
        LinearSlidePower = -this.gamepad2.left_stick_y;
        LinearSlide.setPower(LinearSlidePower);
        telemetry.addData("LinearSlide", LinearSlidePower);
        telemetry.addData("LinearSlide Motor Power", LinearSlide.getPower());
    }

    private void runServo() {
        double armPower = 0;
        armPower = -this.gamepad2.left_stick_x;
        servoArm.setPower(1.25 * armPower);
        telemetry.addData("Arm power", armPower);
        telemetry.addData("Servo Power", servoArm.getPower());
        telemetry.update();
    }

   private void moveDriveTrain() {
       double vertical = 0;
       double horizontal = 0;
       double pivot = 0;
       vertical = -gamepad1.left_stick_y;
       horizontal = gamepad1.left_stick_x;
       pivot = gamepad1.right_stick_x;
       RFMotor.setPower(pivot + (-vertical + horizontal));
       RBMotor.setPower(pivot + -vertical - horizontal);
       LFMotor.setPower(-pivot + -vertical - horizontal);
       LBMotor.setPower(-pivot + (-vertical + horizontal));
   }


    private void moveDriveTrainSlow(){
        if (gamepad1.left_bumper) {
            double vertical = 0;
            double horizontal = 0;
            double pivot = 0;
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            RFMotor.setPower(0.5*(pivot + (-vertical + horizontal)));
            RBMotor.setPower(0.5*(pivot + -vertical - horizontal));
            LFMotor.setPower(0.5*(-pivot + -vertical - horizontal));
            LBMotor.setPower(0.5*(-pivot + (-vertical + horizontal)));

        } else {
            double vertical = 0;
            double horizontal = 0;
            double pivot = 0;
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            RFMotor.setPower(pivot + (-vertical + horizontal));
            RBMotor.setPower(pivot + -vertical - horizontal);
            LFMotor.setPower(-pivot + -vertical - horizontal);
            LBMotor.setPower(-pivot + (-vertical + horizontal));

        }
    }
    double initialPos = 0;
    private double twoBarLiftEncSpeed(double targetPosition, double maxSpeed){
        double difference = targetPosition + initialPos - twoBarLift.getCurrentPosition();
        telemetry.addData("difference", difference);
        double power = Range.clip(-difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

    private void twoBarLift(){
        if (targetPosTwoBarLift > 0  && -this.gamepad2.right_stick_y > 0){
        }
        else if (targetPosTwoBarLift < -1800 && -this.gamepad2.right_stick_y < 0){
        }
        else {
            targetPosTwoBarLift += -this.gamepad2.right_stick_y*15;
            telemetry.addData("Status", "hi" );
        }
    }

    private double linearSlideEncSpeed(double targetPosition, double maxSpeed){
        double difference = targetPosition + initialPos - LinearSlide.getCurrentPosition();
        telemetry.addData("difference", difference);
        double power = Range.clip(-difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

   /*private void runLinearSlide(){
       if (targetPosLinearSlide > 0  && -this.gamepad2.right_stick_y > 0){
       }
       else if (targetPosLinearSlide < -1800 && -this.gamepad2.right_stick_y < 0){
       }
       else {
           targetPosTwoBarLift += -this.gamepad2.right_stick_y*15;
           telemetry.addData("Status", "hi" );
       }
   }*/


    private void clamp(){
        double clampPower = 0;
        if (gamepad2.right_bumper) {
            clamper.setPosition(0.5);
        } else {
            clamper.setPosition(0);

        }
    }

    @Override
    public void runOpMode () {
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        servoArm = hardwareMap.get(CRServo.class, "servoArm");
        clamper = hardwareMap.get(Servo.class, "clamper");
        twoBarLift = hardwareMap.get(DcMotor.class, "twoBarLift");
        LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        initialPos = twoBarLift.getCurrentPosition();
        //linearSlideInitPos = LinearSlide.getCurrentPosition();
        telemetry.addData("Initial Position", initialPos);
        telemetry.update();

        // Put initialization blocks here.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            moveDriveTrain();
            runIntake();
            runOutake();
            runServo();
            runLinearSlide();
            moveDriveTrainSlow();
            twoBarLift();
            clamp();
            //twoBarLiftFast();

            if (gamepad2.b){
                targetPosTwoBarLift = -1684;
            }
            else if (gamepad2.x){
                //targetPosTwoBarLift = -96.0;
                targetPosTwoBarLift = initialPos;
            }
            else if (gamepad2.a){
                targetPosTwoBarLift = -1800;
            }
            else if (gamepad2.y){
                targetPosTwoBarLift = -1200;
            }


            //LinearSlide.setPower(LinearSlideEncSpeed(targetPosLinearSlide, 0.75));
            twoBarLift.setPower(twoBarLiftEncSpeed(targetPosTwoBarLift, 0.75));
            telemetry.addData("encoderTwoBar", twoBarLift.getCurrentPosition() - initialPos);
            telemetry.addData("Target Position Double", targetPosTwoBarLift);


            // Put loop blocks here.
            telemetry.update();

        }
    }

}
