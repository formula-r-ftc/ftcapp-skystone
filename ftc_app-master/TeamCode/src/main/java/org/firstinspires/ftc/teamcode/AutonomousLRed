import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
@Autonomous
@Disabled
public class AutonomousLRed extends LinearOpMode {

    // naming the motors
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private Servo Dumper;
    private RevColorSensorV3 sensorColor;
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static final String VUFORIA_KEY = "";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    static final double FORWARD_SPEED = -1.0;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    private void intakeSkystone() {
        IntakeL.setPower(-0.5*FORWARD_SPEED);
        IntakeR.setPower(0.5*FORWARD_SPEED);
    }

    private void dumpSkystone() {
        Dumper.setPosition(0.25);
    }

    private void connectToHardwareMap() {
        //connecting program names to hardware map names on robot controller phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        Dumper  = hardwareMap.get(Servo.class, "Dumper");
        sensorColor = hardwareMap.get(RevColorSensorV3.class, "Color");
        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void moveForward() {
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }
    private void moveForwardSlow() {
        RFMotor.setPower(0.5*FORWARD_SPEED);
        LFMotor.setPower(0.5*FORWARD_SPEED);
        RBMotor.setPower(0.5*FORWARD_SPEED);
        LBMotor.setPower(0.5*FORWARD_SPEED);
    }

    private void moveUpLeftDiagonal() {
        RFMotor.setPower(FORWARD_SPEED);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void moveLeft() {
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void turnLeft() {
        LFMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
        RFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
    }

    private void moveBackLeftDiagonal() {
        LFMotor.setPower(-FORWARD_SPEED);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setPower(-FORWARD_SPEED);
    }

    private void moveBackward() {
        RFMotor.setPower(-FORWARD_SPEED);
        LFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
    }
    private void moveBackwardSlow() {
        RFMotor.setPower(0.5*-FORWARD_SPEED);
        LFMotor.setPower(0.5*-FORWARD_SPEED);
        RBMotor.setPower(0.5*-FORWARD_SPEED);
        LBMotor.setPower(0.5*-FORWARD_SPEED);

    }

    private void Outake() {
        IntakeL.setPower(-FORWARD_SPEED);
        IntakeR.setPower(-FORWARD_SPEED);
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

    private void moveRight() {
        RFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
    }

    private void turnRight() {
        LFMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
        RFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
    }

    private boolean senseSkyStoneAndCollect() {

        boolean success=false;
        while (opModeIsActive()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# objects detected", updatedRecognitions.size());

                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("left, top(%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (updatedRecognitions != null) {
                            runtime.reset();
                            if (recognition.getLabel().equals(LABEL_SKYSTONE)) {
                                while (opModeIsActive() && runtime.seconds() < 0.2) {
                                    moveBackward();
                                }
                                runtime.reset();
                                while (opModeIsActive() && runtime.seconds() < 0.25) {
                                    moveRight();
                                }
                                runtime.reset();
                                while (opModeIsActive() && runtime.seconds() < 0.1) {
                                    moveForward();
                                }
                                while (opModeIsActive() && runtime.seconds() < 1) {
                                    intakeSkystone();
                                }
                                success = true;
                            }

                        }
                    }
                    if (tfod != null) {
                        tfod.shutdown();
                    }

                }
            }
        }
        return success;
    }
    private void initTensorflow(){
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    @Override

    public void runOpMode() {
        connectToHardwareMap();
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTensorflow();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        if (tfod != null)
            tfod.activate();

        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .25) {
            moveRight();
        }
        boolean success=false;
        do {
            success = senseSkyStoneAndCollect();
            if (success)
                break;
            runtime.reset();
            while (runtime.seconds() < 0.2){
                moveBackward();

            }
        } while (!success);

        //move left
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .25) {
            moveLeft();
        }
        runtime.reset();
        while (opModeIsActive() && (hsvValues[0] < 63.0 || hsvValues[0] > 94.0 )){
            colorSense();
            moveForwardSlow();
        }
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            Outake();
        }
        while (opModeIsActive() && runtime.seconds() < 0.15) {
            //Outake();
        }
        while (opModeIsActive() && (hsvValues[0] < 63.0 || hsvValues[0] > 94.0 )){
            colorSense();
            moveBackwardSlow();
        }
    }
}
