package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class LeftRColorSense extends LinearOpMode {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private ColorSensor ColorSensor;
    private ElapsedTime t1 = new ElapsedTime();
    static final double FORWARD_SPEED = -0.25;

    float hsvValues[] = {0F, 0F, 0F};

    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;

    private void moveLeft() {
        RFMotor.setPower (FORWARD_SPEED);
        LFMotor.setPower (-FORWARD_SPEED);
        RBMotor.setPower (-FORWARD_SPEED);
        LBMotor.setPower (FORWARD_SPEED);
    }
    private void colorSense(){
        Color.RGBToHSV((int) (ColorSensor.red() * SCALE_FACTOR),
                (int) (ColorSensor.green() * SCALE_FACTOR),
                (int) (ColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        telemetry.addData("Alpha", ColorSensor.alpha());
        telemetry.addData("Red  ", ColorSensor.red());
        telemetry.addData("Green", ColorSensor.green());
        telemetry.addData("Blue ", ColorSensor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
    }

    @Override
    public void runOpMode(){

        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        ColorSensor = hardwareMap.get(ColorSensor.class, "Color");

        telemetry.addData("Status" , "Intialized");
        telemetry.update();

        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive() && t1.seconds() < 6){
            RFMotor.setPower(0);
            LFMotor.setPower(0);
            RBMotor.setPower(0);
            LBMotor.setPower(0);

        }
        while (opModeIsActive() && (hsvValues[0] < 15.0 || hsvValues[0] > 35.0)){

            colorSense();
            moveLeft();

        }
    }
}
