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

public class ThreeStoneBlue extends OpMode{


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
    private ElapsedTime t9 = new ElapsedTime();
    private ElapsedTime t12 = new ElapsedTime();

    static final double one = 746.66666666;
    static final double two = 2 * 746.66666666;
    static final double three = 3 * 746.66666666;
    static final double four = 4 * 746.66666666;
    static final double FORWARD_SPEED = 0.2;
    static final double FORWARD_SPEED_FAST = -0.5;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


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
        double avgEncPosition = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
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


    double lastAngle = 0;
    double dAngle = 0;
    private double getHeading() {
        return angles.firstAngle;
    }

    private void rampUp(double length, double heading, double time, double maxSpeed) {
        double AccelerationSlope = (length/Math.abs(length))*(maxSpeed / time);
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

        double AccelerationSlope =(length/Math.abs(length))*(maxSpeed / time);
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeedSide(length, maxSpeed))) {
            driveSideways(turn(heading), power);
        } else {
            driveSideways(turn(heading), encoderSpeedSide(length, maxSpeed));

        }

    }

    boolean tripLoopSidewaysDone = false;
    private boolean tripLoopSideways(double length){
        double avgEncPosition = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;

        if (!tripLoopSidewaysDone && Math.abs(length - avgEncPosition) < 100) {
            tripLoopSidewaysDone = true;
            t2.reset();
        }
        if (tripLoopSidewaysDone && t2.seconds() > 0.75) {
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
        if (tripLoopDone && t2.seconds() > 0.75 ) {
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
        if (tripLoopTurnDone && t2.seconds() > 0.75) {
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

    boolean reset3 = false;
    boolean clampDown = false;
    boolean clampUp = false;
    boolean resetA4 = true;

    private void moveClampsDown(){

        while (t4.seconds() < 0.75){
            clamperL.setPosition(0.0);
            clamperR.setPosition(0.75);
            resetA4 = false;
        }
        if (!resetA4)
            clampDown = true;
    }

    boolean resetA5 = true;
    private void moveClampsUp(){

        while (t5.seconds() < 0.75){
            clamperL.setPosition(0.75);
            clamperR.setPosition(0.0);
            resetA5 = false;
        }
        if (!resetA5)
            clampUp = true;
    }

    private int m_skyStonePattern = 0;
    private int detectSkyStonePattern() {
        return 3;
    }

    boolean grab = false;
    private void grabSkystone() {
        stoneArmClampL.setPosition(1.0);
        if (t8.seconds() > 0.75){
            grab = true;
        }
    }

    boolean grabTwo = false;
    private void grabSkystoneTwo() {
        stoneArmClampL.setPosition(1.0);
        if (t8.seconds() > 0.75){
            grabTwo = true;
        }
    }

    boolean MoveArmDown = false;

    private void moveArmDown() {
        stoneArmL.setPosition(1.0);
        stoneArmClampL.setPosition(0.5);
        if (t12.seconds() > 0.65){
            MoveArmDown = true;
        }

    }

    boolean MoveArmDownTwo = false;

    private void moveArmDownTwo() {
        stoneArmL.setPosition(1.0);
        stoneArmClampL.setPosition(0.5);
        if (t6.seconds() > 0.65){
            MoveArmDownTwo = true;
        }
    }

    private boolean drop = false;

    private void dropSkystone() {
        stoneArmL.setPosition(0.8);
        if (t9.seconds() > 0.7){
            drop = true;
        }
    }
    private boolean dropA = false;
    private void dropSkystoneFully() {
        stoneArmClampL.setPosition(0.3);
        if (t9.seconds() > 0.5){
            dropA = true;
        }
    }

    private boolean dropTwo = false;
    private void dropSkystoneTwo() {
        stoneArmL.setPosition(0.8);
        if (t9.seconds() >0.75){
            dropTwo = true;
        }
    }

    private boolean dropFullyTwo = false;
    private void dropSkystoneFullyTwo() {
        stoneArmClampL.setPosition(0.3);
        if (t9.seconds() > 0.5){
            dropFullyTwo = true;
        }
    }

    private boolean safe = false;

    private void keepSkystone() {
        stoneArmL.setPosition(0.65);
        stoneArmClampL.setPosition(1.0);
        safe = true;
    }

    private boolean safeTwo = false;

    private void keepSkystoneTwo() {
        stoneArmL.setPosition(0.65);
        stoneArmClampL.setPosition(1.0);
        safeTwo = true;
    }

    boolean up = false;
    private void keepArmUp() {
        stoneArmL.setPosition(0.5);
        up = true;
    }

    boolean upTwo = false;
    private void keepArmUpTwo() {
        stoneArmL.setPosition(0.5);
        upTwo = true;
    }

    private void skystoneFinder(){

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
        offset = 0.7;

        m_skyStonePattern = detectSkyStonePattern();

        moveBack1 = moveBack1 + (offset * (m_skyStonePattern-1));
        moveForward1 = moveForward1 + (offset *(m_skyStonePattern-1));
        moveBack2 = moveBack2 + (offset * (m_skyStonePattern-1));
        moveForward2 = moveForward2 + (offset * (m_skyStonePattern-1));

    }
    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

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
        t12.reset();
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
    private boolean trip15 = false;
    private boolean trip16 = false;
    private boolean trip17 = false;
    private boolean trip18 = false;
    private boolean grabSkystone = false;
    private boolean dropSkystone = false;
    private boolean grabSkystone2 = false;
    private boolean dropSkystone2 = false;
    private boolean senseBlue = false;
    private boolean SkystoneFound = false;

    double moveBack1 = 1.5/13.0;
    double moveForward1 = 70.0/13.0;
    double moveBack2 = 86.0/13.0;
    double moveForward2 = 94.0/13.0;
    double offset = 0.7;




    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        //First Skystone
        if (!MoveArmDown){
            moveArmDown();
            if (MoveArmDown){
                t1.reset();
            }
        }
        else if (!SkystoneFound){
            if (detectSkyStonePattern() == 3) {
                rampUp(one * 2 * 0.7, 0, 0.5, 0.5);
                SkystoneFound = tripLoop(one * 2 * 0.7);
            }
            else if (detectSkyStonePattern() == 2){
                rampUp(one*0.7, 0, 0.5, 0.5);
                SkystoneFound = tripLoop(one*0.7);
            }
            else {
                SkystoneFound = true;
            }
            if (SkystoneFound){
                t1.reset();
            }
        }
        else if (!trip1) {
            rampUpSide(one*2.45, 0.0, 0.75, 0.5);
            trip1 = tripLoopSideways(one*2.45);
            if (trip1){
                t8.reset();
            }
        }
        else if (!grab) {
            grabSkystone();
        }
        else if (!safe) {
            keepSkystone();
            if (safe){
                t1.reset();
            }
        }
        else if (!trip2) {
            rampUpSide(-one*0.55, 0.0, 0.75, 0.6);
            trip2 = tripLoopSideways(-one*0.55);
            if (trip2){
                t1.reset();
            }
        }
        else if (!trip3) {
            rampUp(-((one*moveForward1)-1000), 0.0, 0.75, 0.7);
            trip3 = tripLoop(-((one*moveForward1)-1000));
            if(trip3){
                t9.reset();
            }
        }
        else if (!drop) {
            dropSkystone();
            if (drop){
                t9.reset();
            }
        }
        else if (!dropA){
            dropSkystoneFully();
        }
        else if (!up){
            keepArmUp();
            if(up){
                t1.reset();
            }
        }

        //Second Skystone
        else if (!trip4) {
            if (detectSkyStonePattern() == 1) {
                rampUp(one * 4.5 + offset, 0.0, 0.5, 0.7);
                trip4 = tripLoop(one * 4.5 + offset);
            }
            if (detectSkyStonePattern() == 2){
                rampUp(one * 5.5 + offset, 0.0, 0.5, 0.6);
                trip4 = tripLoop(one * 5.5 + offset);
            }
            if (detectSkyStonePattern() == 3){
                rampUp(one * 6.5 + offset, 0.0, 0.5, 0.6);
                trip4 = tripLoop(one * 6.5 + offset);
            }
            if (trip4){
                t6.reset();
            }
        }
        else if (!MoveArmDownTwo) {
            moveArmDownTwo();
            if (MoveArmDownTwo){
                t1.reset();
            }
        }
        else if (!trip5){
            rampUpSide(one*1.65, 0.0, 0.5, 0.5);
            trip5 = tripLoopSideways(one*1.65);
            if (trip5){
                t8.reset();
            }
        }
        else if (!grabTwo) {
            grabSkystoneTwo();
        }
        else if (!safeTwo) {
            keepSkystoneTwo();
            if(safeTwo){
                t1.reset();
            }
        }
        else if (!trip6){
            rampUpSide(-one*0.75, 0, 0.5, 0.5);
            trip6 = tripLoopSideways(-one*0.75);
            if (trip6){
                t1.reset();
            }
        }
        else if (!trip7) {
            if (detectSkyStonePattern() == 1) {
                rampUp(-(one * 4.3 + offset), 0.0, 0.5, 0.7);
                trip7 = tripLoop((-(one * 4.3 + offset)));
            } else if (detectSkyStonePattern() == 2) {
                rampUp(-(one * 5.3 + offset), 0.0, 0.5, 0.7);
                trip7 = tripLoop((-(one * 5.3 + offset)));
            }
            else if (detectSkyStonePattern() == 3){
                rampUp(-(one * 6.3 + offset), 0.0, 0.5, 0.7);
                trip7 = tripLoop((-(one * 6.3 + offset)));
            }
            if(trip7){
                t1.reset();
            }
        }
        else if (!dropTwo) {
            dropSkystoneTwo();
            if(dropTwo){
                t9.reset();
            }
        }
        else if (!dropFullyTwo) {
            dropSkystoneFullyTwo();
        }
        else if (!upTwo){
            keepArmUpTwo();
            if(upTwo){
                t1.reset();
            }
        }
        else if (!trip8){
            rampUp(one*0.85, 0, 0.5, 0.6);
            trip8 = tripLoop(one*0.85);
        }
        else{
            rampUp(0.0, 0, 0.0, 0.1);
        }

        telemetry.addData("Avg Encoder Position", ((LFMotor.getCurrentPosition() - LFPreviousValue) + (RBMotor.getCurrentPosition() - RBPreviousValue) - (RFMotor.getCurrentPosition() - RFPreviousValue) - (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4);
        telemetry.addData("Heading: ", getHeading());
        telemetry.update();
    }

}