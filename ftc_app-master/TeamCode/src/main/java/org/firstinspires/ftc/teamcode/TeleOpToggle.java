package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOpToggle extends LinearOpMode {
    //Naming Objects For Use in Program
    private DcMotor RFMotor;
    private Servo BlockPusher;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private DcMotor slidePlacer;
    private DcMotor LinearSlide;
    private CRServo grabbingServo;
    private DcMotor twoBarLift;
    private Servo clamper;
    private Servo clamperA;
    private Servo clamperB;
    private Servo stoneArmL;
    private Servo stoneArmClampL;
    private Servo stoneArmR;
    private Servo stoneArmClampR;


    //Creating Timers
    private ElapsedTime t1 = new ElapsedTime();
    private ElapsedTime t2 = new ElapsedTime();
    private ElapsedTime t3 = new ElapsedTime();
    private ElapsedTime t4 = new ElapsedTime();
    private ElapsedTime t5 = new ElapsedTime();
    private ElapsedTime t6 = new ElapsedTime();
    private ElapsedTime t7 = new ElapsedTime();
    private ElapsedTime t8 = new ElapsedTime();
    private ElapsedTime t9 = new ElapsedTime();
    private ElapsedTime t12 = new ElapsedTime();
    private ElapsedTime t15 = new ElapsedTime();

    //the vertical position of the two bar lift
    static final double verticalPos = -875;


    //the y position of the two bar lift
    static final double twoBarY = -1250;

    static final double twoBarFoundation = -2100;

    static final double twoBarSecond = -1600;

    static final double twoBarThird = -1400;

    //Doubles that set the input for how far the encoder should go (target positions for values that are entirely global variable based
    private double targetPosTwoBarLift = 0;
    private double targetPosLinearSlide = 0;

    // Double that sets what level the slides go to
    private double autoLevelTarget = 0;

    //Boolean that controls whether or not the intake is running w/ a toggle switch
    boolean toggleIntakeBoolean = false;
    boolean toggleOutakeBoolean = false;

    private void moveDriveTrain(){
        if (gamepad1.left_bumper) {
            double vertical = 0;
            double horizontal = 0;
            double pivot = 0;
            vertical = -gamepad1.left_stick_y;
            horizontal = gamepad1.left_stick_x;
            pivot = gamepad1.right_stick_x;
            RFMotor.setPower(0.3*(pivot + (-vertical + horizontal)));
            RBMotor.setPower(0.3*(pivot + -vertical - horizontal));
            LFMotor.setPower(0.3*(-pivot + -vertical - horizontal));
            LBMotor.setPower(0.3*(-pivot + (-vertical + horizontal)));


        }
        else {
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
        telemetry.addData("Two Bar Difference", difference);
        double power = Range.clip(-difference/ 500, -maxSpeed, maxSpeed);
        return power;
    }

    double linearSlideInitPos = 0;
    private double linearSlideEncSpeed(double targetPosition, double maxSpeed){
        double difference = targetPosition + linearSlideInitPos - LinearSlide.getCurrentPosition();
        telemetry.addData("LS Difference", difference);
        double power = Range.clip(-difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

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
        if (toggleIntakeBoolean){
            IntakeL.setPower(0.5);
            IntakeR.setPower(-0.5);
        }
        else if (toggleOutakeBoolean){
            IntakeL.setPower(-0.5);
            IntakeR.setPower(0.5);

        }
        else if (!toggleIntakeBoolean && !toggleOutakeBoolean){
            IntakeL.setPower(0.0);
            IntakeR.setPower(0.0);
        }
    }

    private void OutakeToggle(){
        if (gamepad2.right_trigger > 0.5 && t1.seconds() > 0.5 ){
            t1.reset();
            toggleOutakeBoolean=true;
            toggleIntakeBoolean=false;
        }
    }

    boolean toggleTwoBarBoolean = false;
    boolean twoBarPosA = false;
    private void AutoLevelTwoBar(){
        if (gamepad2.y && t7.seconds() > 0.5 ){
            t7.reset();
            if (!toggleTwoBarBoolean){
                twoBarPosA = true;
                toggleTwoBarBoolean=true;
            } else if (toggleTwoBarBoolean){
                twoBarPosA = false;
                toggleTwoBarBoolean=false;
            }
        }
    }

    private void clamp(){
        if (gamepad2.right_bumper) {
            clampClosed = true;
        }
    }

    private void clampFoundationA(){
        if (gamepad1.right_bumper) {
            clamperA.setPosition(0.0);
        } else {
            clamperA.setPosition(0.5);
        }
    }

    boolean clampClosed = true;
    private void clampFoundationB(){
        if (gamepad1.right_bumper) {
            clamperB.setPosition(0.5);
        } else {
            clamperB.setPosition(0.0);
        }
    }
    boolean placementMode = false;
    private void toggle(){
        if ( gamepad2.b && t4.seconds() > 1.0 ){
            t4.reset();
            if (placementMode == true){
                placementMode = false;
            }
            else if (placementMode == false){
                placementMode = true;
            }
        }
    }

    private void autoLevel() {
        if (placementMode) {
            clampClosed = true;
            if (autoLevelTarget > 3) {
                targetPosLinearSlide = (((autoLevelTarget - 3) * -500) - 400);
                if (twoBarPosA) {
                    targetPosTwoBarLift = twoBarY;
                } else {
                    targetPosTwoBarLift = verticalPos;
                }
            }
            else if (autoLevelTarget == 3) {
                targetPosLinearSlide = -2000;
                if (twoBarPosA == true) {
                    targetPosTwoBarLift = (twoBarFoundation-100);
                } else {
                    targetPosTwoBarLift = verticalPos;
                }
            }
            else if (autoLevelTarget == 2) {
                targetPosLinearSlide = -1600;
                if (twoBarPosA) {
                    targetPosTwoBarLift = (twoBarFoundation-50);
                } else {
                    targetPosTwoBarLift = verticalPos;
                }
            }
            else if (autoLevelTarget == 1) {
                targetPosLinearSlide = -1050;
                if (twoBarPosA == true) {
                    targetPosTwoBarLift = twoBarFoundation;
                } else {
                    targetPosTwoBarLift = verticalPos;
                }
            }
        }
        else {
            targetPosTwoBarLift = -70;
            targetPosLinearSlide = 0;
            twoBarPosA = false;
            clampClosed = false;
            clamp();
        }
    }
    double depositValue = 400;
    double releaseTime = 1;
    boolean buttonPressed = false;
    private void depositBlock(){
        if (placementMode == true && gamepad2.a){
            if (buttonPressed == false){
                t8.reset();
                buttonPressed=true;
            }
            double slope = (depositValue/releaseTime);
            if ((t8.seconds()*slope) < 400){
                targetPosLinearSlide += t8.seconds()*slope;
            }
            else {
                targetPosLinearSlide += 400;
            }
            if (t8.seconds() > 1.0){
                clampClosed = false;
                t9.reset();
            }
        }
        else {
            buttonPressed = false;
            if (placementMode && t9.seconds() < 5.0){
                clampClosed = false;
                if (t9.seconds() > 0.25){
                    targetPosTwoBarLift = verticalPos;
                }
            }
        }

    }
    private void linearSlideUpIncrements(){
        if ( gamepad2.dpad_up && t2.seconds() > 0.5 ){
            t2.reset();
            autoLevelTarget++;
        }
    }

    private void linearSlideDownIncrements(){
        if (gamepad2.dpad_down && t3.seconds() > 0.5 ){
            t3.reset();
            autoLevelTarget--;
        }
    }
    int twoBarTarget = 0;
    private void twoBarUpIncrements(){
        if (gamepad2.dpad_right){
            t12.reset();
            twoBarTarget++;
        }
    }
    private void twoBarDownIncrements(){
        if (gamepad2.dpad_left){
            t15.reset();
            twoBarTarget--;
        }
    }

    private void twoBarManual(){
        targetPosTwoBarLift +=(twoBarTarget*-60);
    }


    @Override
    public void runOpMode () {
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        clamper = hardwareMap.get(Servo.class, "clamper");
        twoBarLift = hardwareMap.get(DcMotor.class, "twoBarLift");
        LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        clamperA = hardwareMap.get(Servo.class, "clamperL");
        clamperB = hardwareMap.get(Servo.class, "clamperR ");
        stoneArmClampL = hardwareMap.get(Servo.class, "stoneArmClampL");
        stoneArmL = hardwareMap.get(Servo.class, "stoneArmL");
        stoneArmClampR = hardwareMap.get(Servo.class, "stoneArmClampR");
        stoneArmR = hardwareMap.get(Servo.class, "stoneArmR");
        BlockPusher = hardwareMap.get(Servo.class, "BlockPusher");
        initialPos = twoBarLift.getCurrentPosition();
        linearSlideInitPos = LinearSlide.getCurrentPosition();
        telemetry.addData("Two Bar Position", twoBarLift.getCurrentPosition());
        telemetry.update();

        // Put initialization blocks here.
        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            moveDriveTrain();
            clampFoundationA();
            clampFoundationB();
            IntakeToggle();
            OutakeToggle();
            IntakeButtons();
            intakeControl();
            linearSlideUpIncrements();
            linearSlideDownIncrements();
            twoBarUpIncrements();
            twoBarDownIncrements();
            toggle();
            autoLevel();
            AutoLevelTwoBar();
            twoBarManual();
            if (targetPosLinearSlide < -3600 ){
                targetPosLinearSlide = -3600;
            }
            depositBlock();
            clamp();
            stoneArmL.setPosition(0.45);
            stoneArmClampL.setPosition(1.0);
            stoneArmR.setPosition(0.68);
            stoneArmClampR.setPosition(1.0);
            if (clampClosed) {
                clamper.setPosition(0.5);
            }
            else {
                clamper.setPosition(0);
            }
            if (targetPosLinearSlide > 0 ){
                targetPosLinearSlide = 0;
            }

            if (targetPosTwoBarLift > 0){
                targetPosTwoBarLift = 0;
            }
            if (targetPosTwoBarLift < -2300){
                targetPosTwoBarLift = -2300;
            }
            if (toggleOutakeBoolean){
                BlockPusher.setPosition(0.3);
            }
            else {
                BlockPusher.setPosition(0.0);
            }
            LinearSlide.setPower(linearSlideEncSpeed(targetPosLinearSlide, 0.75));
            twoBarLift.setPower(twoBarLiftEncSpeed(targetPosTwoBarLift, 0.75));

            telemetry.addData("Linear Slide Position", LinearSlide.getCurrentPosition());
            telemetry.addData("Two Bar Position", twoBarLift.getCurrentPosition());
            telemetry.addData("Target Position - Linear Slide", targetPosLinearSlide);
            telemetry.addData("Target Position - Two Bar Lift", targetPosTwoBarLift);
            telemetry.addData("AutoLevelTarget", autoLevelTarget);
            telemetry.addData("Placement Mode", placementMode);
            telemetry.update();
        }
    }
}