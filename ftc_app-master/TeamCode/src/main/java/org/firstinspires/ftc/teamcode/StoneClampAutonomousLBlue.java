package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous

public class StoneClampAutonomousLBlue extends OpMode{

    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private ColorSensor sensorColor;
    private Servo clamperL;
    private Servo clamperR;
    private Servo stoneArmL;
    private Servo stoneArmClampL;


    private ElapsedTime t1 = new ElapsedTime();
    private ElapsedTime t2 = new ElapsedTime();
    private ElapsedTime t3 = new ElapsedTime();
    private ElapsedTime t4 = new ElapsedTime();
    private ElapsedTime t5 = new ElapsedTime();
    private ElapsedTime t6 = new ElapsedTime();
    private ElapsedTime t7 = new ElapsedTime();
    private ElapsedTime t8 = new ElapsedTime();

    static final double one = 746.66666666;
    static final double onepointfive =1.5*746.66666666;
    static final double two = 2 * 746.66666666;
    static final double three = 3 * 746.66666666;
    static final double four = 4 * 746.66666666;
    static final double righthalf = -373.33333333;
    static final double FORWARD_SPEED = 0.2;
    static final double FORWARD_SPEED_FAST = -0.5;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    private void moveToPosition(double targetPosition) {
        double difference = targetPosition - RFMotor.getCurrentPosition();
        telemetry.addData("difference", difference);
        setTankPower(-difference / 500);
    }

    private void moveOne(DcMotor a, double targetPosition) {
        a.getCurrentPosition();
        double difference = targetPosition - a.getCurrentPosition();
        telemetry.addData("difference", difference);
        double power = Range.clip(-difference / 250, -0.5, 0.5);
        a.setPower(power);
    }

    private double turn(double targetAngle) {

        double current = getHeading();
        double difference = targetAngle - current;
        telemetry.addData("difference in turn is", difference);
        double power = Range.clip(difference / 45, -0.5, 0.5);
        return power;

    }

    private double encoderSpeed(double targetEncoderValue, double maxSpeed) {
        double avgEncPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double difference = targetEncoderValue - avgEncPosition;
        telemetry.addData("difference", difference);
        double power = Range.clip(difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

    private double headingSpeed(double targetHeadingValue, double maxSpeed) {
        double difference = targetHeadingValue - getHeading();
        telemetry.addData("difference", difference);
        double power = Range.clip(difference / 50, -maxSpeed, maxSpeed);
        return power;
    }

    private double encoderSpeedSide(double targetPosition, double maxSpeed) {
        double avgEncPosition = ((LFMotor.getCurrentPosition() - LFPreviousValue) + (RBMotor.getCurrentPosition() - RBPreviousValue) - (RFMotor.getCurrentPosition() - RFPreviousValue) - (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
        double difference = targetPosition - avgEncPosition;
        telemetry.addData("difference", difference);
        double power = Range.clip(difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

    private void setTurnPower(double turnPower, double power) {
        RFMotor.setPower(-turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(-turnPower - power);
        LBMotor.setPower(turnPower - power);
        telemetry.addData("turn power", turnPower);
    }

    private void driveSideways(double turnPower, double encoderSpeedSide) {
        RFMotor.setPower(-turnPower - encoderSpeedSide);
        LFMotor.setPower(turnPower + encoderSpeedSide);
        RBMotor.setPower(-turnPower + encoderSpeedSide);
        LBMotor.setPower(turnPower - encoderSpeedSide);

    }

    private void move(double targetPosition) {
        moveOne(RFMotor, targetPosition);
        moveOne(LFMotor, targetPosition);
        moveOne(RBMotor, targetPosition);
        moveOne(LBMotor, targetPosition);
    }

    private void setTankPower(double power) {
        telemetry.addData("power", power);
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }

    double lastAngle = 0;
    double dAngle = 0;
    private double getHeading() {
        return angles.firstAngle;
    }

    private void rampUp(double length, double heading, double time, double maxSpeed) {
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(length, maxSpeed))) {
            setTurnPower(turn(heading), power);
        } else {
            setTurnPower(turn(heading), encoderSpeed(length, maxSpeed));

        }

    }

    private void rampUpTurn(double heading, double time, double maxSpeed) {
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(headingSpeed(heading, maxSpeed))) {
            setTurnPower((headingSpeed(heading, maxSpeed)/Math.abs(headingSpeed(heading, maxSpeed))*power), 0.0);
            telemetry.addData("Time","Using TIME");
            telemetry.update();
        } else {
            setTurnPower((headingSpeed(heading, maxSpeed)), 0.0);
            telemetry.addData("Heading", "Using HEADING" );
            telemetry.update();
        }

    }

    private void rampUpSide(double length, double heading, double time, double maxSpeed) {

        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeedSide(length, maxSpeed))) {
            driveSideways(turn(heading), power);
        } else {
            driveSideways(turn(heading), encoderSpeedSide(length, maxSpeed));

        }

    }

    boolean tripLoopSidewaysDone = false;
    private boolean tripLoopSideways(double length){
        double avgEncPosition = ((LFMotor.getCurrentPosition() - LFPreviousValue) + (RBMotor.getCurrentPosition() - RBPreviousValue) - (RFMotor.getCurrentPosition() - RFPreviousValue) - (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;

        if (!tripLoopSidewaysDone && Math.abs(length - avgEncPosition) < 100) {
            tripLoopSidewaysDone = true;
            t2.reset();
        }
        if (tripLoopSidewaysDone && t2.seconds() >1 ) {
            tripLoopSidewaysDone = false;
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = LFMotor.getCurrentPosition();
            LBPreviousValue = LBMotor.getCurrentPosition();
            return true;
        }
        return false;
    }

    boolean tripLoopDone = false;
    private boolean tripLoop(double length) {
        double avgEncPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition()  - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;

        if (!tripLoopDone && (Math.abs(length - avgEncPosition) < 100)) {
            tripLoopDone = true;
            t2.reset();
        }
        if (tripLoopDone && t2.seconds() > 1 ) {
            tripLoopDone = false;
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            return true;
        }
        return false;
    }

    boolean tripLoopTurnDone = false;
    private boolean tripLoopTurn(double turn) {

        if (!tripLoopTurnDone && Math.abs(turn - getHeading()) < 20) {
            tripLoopTurnDone = true;
            t2.reset();
        }
        if (tripLoopTurnDone && t2.seconds() > 1) {
            tripLoopTurnDone = false;
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = LFMotor.getCurrentPosition();
            LBPreviousValue = LBMotor.getCurrentPosition();
            return true;
        }
        else {
            return false;
        }
    }
    private void colorSense() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
    }
    boolean sensedBlue = false;
    boolean ends = false;

    private boolean forwardUntilBlue(){
        if (hsvValues[0] < 195.0 || hsvValues[0] > 225.0){
            colorSense();
            setTurnPower(turn(0.0), 0.25);
        }
        else {
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            sensedBlue = true;
            ends = true;
        }
        return sensedBlue;
    }

    private void moveBackwardSlow() {
        RFMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        LFMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        RBMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        LBMotor.setPower(0.5*-FORWARD_SPEED_FAST);
    }
    boolean reset3 = false;

    boolean clampDown = false;

    boolean clampUp = false;

    boolean resetA4 = true;
    private void moveClampsDown(){
        t4.reset();

        while (t4.seconds() < 0.5){
            clamperL.setPosition(0.5);
            clamperR.setPosition(0.0);
            resetA4 = false;
        }
        if (!resetA4)
            clampDown = true;
    }

    boolean resetA5 = true;
    private void moveClampsUp(){
        t5.reset();

        while (t5.seconds() < 0.5){
            clamperL.setPosition(0.0);
            clamperR.setPosition(0.5);
            resetA5 = false;
        }
        if (!resetA5)
            clampUp = true;
    }

    private int m_skyStonePattern = 0;
    private int detectSkyStonePattern() {

        return 1;

    }

    boolean grab = false;
    private void grabSkystone() {
        stoneArmClampL.setPosition(1.0);
        if (t8.seconds() > 1){
            grab = true;
        }
    }

    boolean MoveArmDown = false;

    private void moveArmDown() {
        stoneArmL.setPosition(0.4);
        stoneArmClampL.setPosition(0.4);

        MoveArmDown = true;
    }

    private boolean drop = false;
    boolean StoneNotReleased = true;

    private void dropSkystone() {
        stoneArmL.setPosition(0.3);
        StoneNotReleased = false;
        if (!StoneNotReleased) {
            stoneArmClampL.setPosition(0.3);
            drop = true;
        }
    }

    private boolean safe = false;

    private void keepSkystone() {
        stoneArmL.setPosition(0.0);
        stoneArmClampL.setPosition(1.0);
        safe = true;
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        msStuckDetectLoop=10000;


        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        sensorColor = hardwareMap.get(ColorSensor.class, "Color");
        clamperL = hardwareMap.get(Servo.class, "clamperL");
        clamperR = hardwareMap.get(Servo.class, "clamperR");
        stoneArmClampL = hardwareMap.get(Servo.class, "stoneArmClampL");
        stoneArmL = hardwareMap.get(Servo.class, "stoneArmL");
        imu = hardwareMap.get(BNO055IMU.class, "emu");

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled=false;

        imu.initialize(parameters);

        moveBack1 = 1.5/13.0;
        moveForward1 = 70.0/13.0;
        moveBack2 = 100.0/13.0;
        moveForward2 = 100.0/13.0;
        offset = 8.0/13.0;

        m_skyStonePattern = detectSkyStonePattern();

        moveBack1 = moveBack1 + (offset * (m_skyStonePattern-1));
        moveForward1 = moveForward1 + (offset *(m_skyStonePattern-1));
        moveBack2 = moveBack2 + (offset * (offset-1));
        moveForward2 = moveForward2 + (offset * (m_skyStonePattern-1));

    }
    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

        moveOne(RFMotor, 0);
        moveOne(LFMotor, 0);
        moveOne(RBMotor, 0);
        moveOne(LBMotor, 0);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();
    }

    @Override
    public void start() {
        t1.reset();
        t2.reset();
        t3.reset();
    }


    private boolean trip1 = false;
    private boolean trip2 = false;
    private boolean trip3 = false;
    private boolean trip4 = false;
    private boolean trip5 = false;
    private boolean trip6 = false;
    private boolean trip7 = false;
    private boolean trip8 = false;
    private boolean trip9 = false;
    private boolean trip10 = false;
    private boolean trip11 = false;
    private boolean trip12 = false;
    private boolean trip13 = false;
    private boolean trip14 = false;
    private boolean grabSkystone = false;
    private boolean dropSkystone = false;
    private boolean grabSkystone2 = false;
    private boolean dropSkystone2 = false;
    private boolean senseBlue = false;

    double moveBack1 = 1.5/13.0;
    double moveForward1 = 70.0/13.0;
    double moveBack2 = 86.0/13.0;
    double moveForward2 = 94.0/13.0;
    double offset = 8.0/13.0;




    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //First Skystone
        if (!MoveArmDown){
            moveArmDown();
        }
        else if (!trip1) {
            rampUpSide(one*2.5, 0.0, 0.5, 0.5);
            trip1 = tripLoopSideways(one*2.5);
            if (trip1){
                t8.reset();
            }
        }
        else if (!grab) {
            grabSkystone();
        }
        else if (!safe) {
            keepSkystone();
        }
        else if (!trip3) {
            rampUpSide(-one*2.1, 0.0, 0.5, 0.5);
            trip3 = tripLoopSideways(-one*2.1);
        }
        else if (!trip4) {
            rampUp(-one*moveForward1, 0.0, 0.5, 0.5);
            trip4 = tripLoop(-one*moveForward1);
            if(trip4){
                t1.reset();
            }
        }
        else if  (!trip5) {
            rampUpSide(-one*0.5, 0.0, 0.5, 0.5);
            trip5 = tripLoopSideways(-one*0.5);
            if(trip5){
                t1.reset();
            }
        }
        else if (!dropSkystone) {
            dropSkystone();
        }
        //Second Skystone
        else if (!trip7) {
            rampUpTurn(0.0, 1.0, 0.5);
            trip7 = tripLoopTurn(0.0);
        }
        else if (!trip10) {
            rampUp(one*moveBack2, 0.0, 0.5, 0.5);
            trip10 = tripLoop(one*moveBack2);
        }
        else if (!grab) {
            grabSkystone();
        }
        else if (!safe) {
            keepSkystone();
        }
        else if (!trip11) {
            rampUpSide(one*1.92, 0.0, 0.5, 0.5);
            trip11 = tripLoopSideways(one*1.92);
        }
        else if (!trip12) {
            rampUp(-one*moveForward2, 0.0, 0.5, 0.5);
            trip12 = tripLoop(-one*moveForward2);
        }
        else if (!trip13) {
            rampUp(-0.0, 90.0, 0.5, -0.5);
            trip13 = tripLoopTurn(90.0);
        }
        else if (!drop) {
            dropSkystone();
        }
        else if (!trip14) {
            rampUp(-0.0, 180, 0.5, -0.2);
            trip14 = tripLoopTurn(180.0);
        }
        else if (!senseBlue) {
            senseBlue = forwardUntilBlue();
        }

        telemetry.addData("Avg Encoder Position", ((LFMotor.getCurrentPosition() - LFPreviousValue) + (RBMotor.getCurrentPosition() - RBPreviousValue) - (RFMotor.getCurrentPosition() - RFPreviousValue) - (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4);
        telemetry.addData("Heading: ", getHeading());
        telemetry.addData("cool", angles.firstAngle);
        telemetry.addData("Timer 3", t3.seconds());
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.addData("dangle", dAngle);
        telemetry.addData("QWR", reset3);
        telemetry.addData("Trip 4",trip4);
        telemetry.addData("Timer 2", t2.seconds());
        telemetry.addData("Ends", ends);
        telemetry.update();
        getHeading();
    }

}

