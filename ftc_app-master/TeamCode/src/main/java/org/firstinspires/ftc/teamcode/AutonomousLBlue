import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
public class AutonomousLBlue extends LinearOpMode {

    // naming the motors
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private DcMotor IntakeL;
    private DcMotor IntakeR;
    private RevColorSensorV3 sensorColor;
    private CRServo servoArm;
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_STONE = "Stone";
    private static final String LABEL_SKYSTONE = "Skystone";

    private static final String VUFORIA_KEY = " AajIlwT/////AAABmVP1K7jCX0QfjJZI1ObT2eMd8NDwQYnfQC6oXBrLo6q0G1BuEH0ncpGWifN+H0vGKqsEnT20jnvrI7jMSnELgNZHViY3x6t9bTQz/3/HdQEsR3Q4sfRSFTOLQWVx1CG3okkbopOA0NGgzww1fbXVgNW+HL8vvtZDOoxhRoZW933xTc48GXvise6oIts9Cw3vUC+CEzC8JOYTSR3LDqfFk0nxHyHm/EbNR+gaxrj3f7y18sxxV9df29YzlzNbwWcLn+6YI9aEhmHBem3P/DZGg7qk+4VpCpLbURUsOgFPIMcbLbaZcau3I+Iooscu3k6O0J0ZP35SPADOnGJOae5ZeKV82XcF4hGOl+W4RQ7DAaRD ";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    static final double FORWARD_SPEED = -0.5;
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

    private void connectToHardwareMap() {
        //connecting program names to hardware map names on robot controller phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        IntakeL = hardwareMap.get(DcMotor.class, "IntakeL");
        IntakeR = hardwareMap.get(DcMotor.class, "IntakeR");
        sensorColor = hardwareMap.get(RevColorSensorV3.class, "Color");
        servoArm = hardwareMap.get(CRServo.class, "servoArm");
        //Sending status to drivers station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    private void moveForward() {
        //All motors move forward 
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
    private void moveUpLeftDiagonal() {
        RFMotor.setPower(FORWARD_SPEED);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void moveBackLeftDiagonal() {
        LFMotor.setPower(-FORWARD_SPEED);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setPower(-FORWARD_SPEED);
    }

    private void moveLeft() {
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }

    private void moveRight() {
        RFMotor.setPower(-FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
    }

    private void turnRight() {
        LFMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
        RFMotor.setPower(-FORWARD_SPEED);
        RBMotor.setPower(-FORWARD_SPEED);
    }


    private void turnLeft() {
        LFMotor.setPower(-FORWARD_SPEED);
        LBMotor.setPower(-FORWARD_SPEED);
        RFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
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

    private void moveArmDown(){
        servoArm.setPower(FORWARD_SPEED);
    }
    
    private void moveArmUp(){
        servoArm.setPower(-FORWARD_SPEED);
    }

    private boolean senseSkyStoneAndCollect() {
        telemetry.addData("Loop", "Entered Sense Skystone and Collect" );
        telemetry.update();
        boolean success=false;
        while (opModeIsActive() && !success) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# objects detected", updatedRecognitions.size());
                    telemetry.update();
                    try {
        Thread.sleep(5000);
        } catch (InterruptedException e) {
            
        }
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("left, top(%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                                
                        if (updatedRecognitions != null) {
                            if (recognition.getLabel().equals(LABEL_SKYSTONE)) {
                             telemetry.addData("Detection","SkyStone detected\n");
                             telemetry.update();
                             success = true;
                            // break;
                                
                            }
                            else {
                             telemetry.addData("Detection"," Regular stone detected\n");
                             telemetry.update();   
                                
                            }
                            /*runtime.reset();
                            if (recognition.getLabel().equals(LABEL_SKYSTONE)) {
                                runtime.reset();
                                while (opModeIsActive() && runtime.seconds() < 1.01) {
                                    moveForward();
                                }
                                runtime.reset();
                                while (opModeIsActive() && runtime.seconds() < 0.23) {
                                    turnLeft();
                                }
                                success = true;
                                break;
                            }*/

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
        tfodParameters.minimumConfidence = 0.60;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_STONE, LABEL_SKYSTONE);
        assert(tfod != null);
        
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;
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
            telemetry.update();
        }
        if (tfod != null)
            tfod.activate();

        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        /*runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.15) {
            moveForward();
        }*/
        runtime.reset();

        boolean success=false;
        do {
            success = senseSkyStoneAndCollect();
            if (success)
                break;
            runtime.reset();
            while (runtime.seconds() < 0.2){
                moveRight();

            }
        } while (!success);
        
       /* runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.75) {
            moveArmDown();
        }*/

       /* runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 0.5){
            moveLeft();
        }*/
        
       /* runtime.reset();
        while (opModeIsActive() && (hsvValues[0] < 63.0 || hsvValues[0] > 94.0 )){
            colorSense();
            moveForwardSlow();
        }
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            moveForward();
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 1.0){
            moveArmUp();
        }
        runtime.reset();
        while (opModeIsActive() && (hsvValues[0] < 63.0 || hsvValues[0] > 94.0 )){
            colorSense();
            moveBackwardSlow();
        }*/
    }
}
