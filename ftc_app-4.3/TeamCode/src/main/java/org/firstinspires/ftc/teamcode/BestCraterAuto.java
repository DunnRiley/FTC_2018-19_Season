package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="last mach", group="Linear Opmode")
public class BestCraterAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private String mineral;

    private DriveUtilities driveUtilities;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AUKN4dL/////AAABmZL4EHQ+ukIzpu89viYzaR4/jtXdbGjrssYSj0jYOGUszrxyjIZgyw6bnlbe3u5cMKxWpBP4i8HPWkAKoqmaJmRHrv8Ho7vPazLYJ65ZoLevtugk9OihzarMGR1FSjcThK3Tx3k2zpgCwaZWGopNS4ObXr5mFqdZkLmE11hEj7QV/sX886rFKB6Q9ySF1L4eYhwZmsArJjf5BGgenPwg0/Ou8yhB3VXk5++EAbLQwwlSFYZnQkS8h9TeGHe7LL5CbPgYptx+qo8iLJqqx6cqkabRHyVv1rzTxeJw42wFpHAono6FcMrWOiuKZs2xBiKKlUlw2qI8cl6sHbhiRPcOtLO3X5p7f0+fInRqOY4pakdS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    boolean chkloop = true;
    DcMotor RLin,LLin, RLift,LLift;

    @Override
    public void runOpMode() {
        RLin = hardwareMap.get(DcMotor.class, "RLin");
        LLin = hardwareMap.get(DcMotor.class, "LLin");
        RLift = hardwareMap.get(DcMotor.class, "RLift");
        LLift = hardwareMap.get(DcMotor.class, "LLift");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driveUtilities = new DriveUtilities(hardwareMap,this);


        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        driveUtilities.hangServo.setPosition(1);
        driveUtilities.LLift.setPower(.1);
        driveUtilities.RLift.setPower(-.1);//OPPOSITE FOR DOWN
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            while (chkloop) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() != 0) {
                            int goldMineralX = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1) {
                                telemetry.addData("Gold ", goldMineralX);
                                telemetry.update();
                            }
                            if (goldMineralX <= 150 && goldMineralX >= 0) {
                                telemetry.addData("Position = ", "Left");
                                telemetry.update();
                                leftGold();
                                chkloop = false;
                                break;
                            }else if (goldMineralX <= 450 && goldMineralX >= 150) {
                                telemetry.addData("Position = ", "Center");
                                telemetry.update();
                                centerGold();
                                chkloop = false;
                                break;
                            }else if (goldMineralX >= 600 && goldMineralX <= 850) {
                                telemetry.addData("Position = ", "Right");
                                telemetry.update();
                                rightGold();
                                chkloop = false;
                                break;
                            }else if (runtime.seconds() >= 2) {
                                telemetry.addData("Timeout", "RIP");
                                telemetry.update();
                                failDrop();
                                chkloop = false;
                                break;
                            }


                        } else if (runtime.seconds() >= 2){
                            telemetry.addData("Timeout","RIP");
                            telemetry.update();
                            failDrop();
                            chkloop = false;
                            break;
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public void drop() {
        driveUtilities.LLift.setPower(-1);
        driveUtilities.RLift.setPower(1);
        while (opModeIsActive() && driveUtilities.LLift.getCurrentPosition() > 1007.2){
            telemetry.addData("Movement:","dropping");
            telemetry.update();
        }
        driveUtilities.RLift.setPower(0);
        driveUtilities.LLift.setPower(0);
        driveUtilities.hangServo.setPosition(1);
        driveUtilities.hangServo.setPosition(1);
        driveUtilities.LLift.setPower(1);
        driveUtilities.RLift.setPower(-1);
        while (opModeIsActive() && driveUtilities.LLift.getCurrentPosition() > 1007.2){
            telemetry.addData("Movement:","dropping");
            telemetry.update();
        }
        driveUtilities.RLift.setPower(0);
        driveUtilities.LLift.setPower(0);


    }
    public void centerGold() {
        drop();
        driveUtilities.drive(1,30,10);
        driveUtilities.drive(-1,-30,10);

    }
    public void leftGold() {
        drop();
        driveUtilities.drive(.3,5,5);
        driveUtilities.CCWRotate(40);
        driveUtilities.drive(.3,30,10);
        driveUtilities.drive(-.3,-30,10);
        driveUtilities.CWRotate(40);
        driveUtilities.drive(-.3,-5,5);
    }
    public void rightGold() {
        drop();
        driveUtilities.drive(.3,5,5);
        driveUtilities.CWRotate(40);
        driveUtilities.drive(.3,30,10);
        driveUtilities.drive(-.3,-30,10);
        driveUtilities.CCWRotate(40);
        driveUtilities.drive(-.3,-5,5);

    }
    public void failDrop(){
        drop();
    }
    public void park(){
        driveUtilities.drive(.3,5,5);
        driveUtilities.CCWRotate(75);
        driveUtilities.drive(.3,30,10);
        driveUtilities.CCWRotate(65);
        driveUtilities.drive(.3,45,10);

    }
}
