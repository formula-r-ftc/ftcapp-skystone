package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class MoveBackwardUntilSensedBlue extends OpMode {

    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;

    private ColorSensor sensorColor;


    private ElapsedTime t1 = new ElapsedTime();
    private ElapsedTime t2 = new ElapsedTime();
    private ElapsedTime t3 = new ElapsedTime();
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
        double difference = targetAngle - getHeading();
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

    private void setTurnPower(double turnPower, double power) {
        RFMotor.setPower(-turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(-turnPower - power);
        LBMotor.setPower(turnPower - power);
        telemetry.addData("turn power", turnPower);
    }

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
    private void backUntilBlue(){
        if (hsvValues[0] < 200.0 || hsvValues[0] > 220.0){
            colorSense();
            setTurnPower(turn(0.0), -0.25);
        }
        else{
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            sensedBlue = true;
            ends = true;
        }
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

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }
    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.update();
    }

    @Override
    public void start() {
        t1.reset();
        t2.reset();
        t3.reset();
    }

    @Override
    public void loop() {
        if(!sensedBlue) {
            backUntilBlue();
        }
        else {
            rampUp(0.0, 0.0, 0.5, 0.5);
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}

