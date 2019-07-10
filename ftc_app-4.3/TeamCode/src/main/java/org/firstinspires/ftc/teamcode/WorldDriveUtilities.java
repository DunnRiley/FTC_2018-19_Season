package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class WorldDriveUtilities {

    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    //Declare Motors and servos
    DcMotor FRD,FLD,BRD,BLD,RLin,LLin,RLift,LLift;
    LinearOpMode opMode;
    //declare constants
    private static final double     ROBOT_DIAMETER          = 16;
    private static final double     DIST_BETWEEN_EDGES      = 12;
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;
    private static final double     ACCELERATION_GRADIENT   = 1.0/200;
    private static final double     DECELERATION_GRADIENT   = 1.0/2000;//rate of acceleration and decceleration of robot
    private static final double     MINIMUM_POWER           = 0.1;
    private static final double     DRIVE_GEAR_REDUCTION    = 2 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public WorldDriveUtilities(HardwareMap hardwareMap, LinearOpMode opMode) {
        //Initialize motors and variables
        FLD = hardwareMap.get(DcMotor.class, "FLD");
        FRD = hardwareMap.get(DcMotor.class, "FRD");
        BLD = hardwareMap.get(DcMotor.class, "BLD");
        BRD = hardwareMap.get(DcMotor.class, "BRD");
        LLin = hardwareMap.get(DcMotor.class, "LLin");
        RLin= hardwareMap.get(DcMotor.class, "RLin");
        RLift = hardwareMap.get(DcMotor.class, "RLift");
        LLift = hardwareMap.get(DcMotor.class, "LLift");

        FRD.setDirection(DcMotor.Direction.REVERSE);
        FLD.setDirection(DcMotor.Direction.REVERSE);
        BRD.setDirection(DcMotor.Direction.REVERSE);
        BLD.setDirection(DcMotor.Direction.FORWARD);


        //Set encoder modes
        FRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        this.opMode = opMode;

    }
    //Function to move the robot on a speed gradient
    private double getPower(double targetPower, int curPos, int endPos, int startingPos){
        double rampUpPower = Math.abs(curPos - startingPos) * ACCELERATION_GRADIENT;
        double rampDownPower = Math.abs(endPos - curPos) * DECELERATION_GRADIENT;
        opMode.telemetry.addData("rampDownPower", rampDownPower);
        opMode.telemetry.addData("rampUpPower", rampUpPower);
        //opMode.telemetry.update();
        double clippedPower = Math.min(Math.min(targetPower, rampUpPower), rampDownPower);
        opMode.telemetry.addData("clipped power ", clippedPower);
        return Math.max(MINIMUM_POWER, clippedPower);  //Ensure power is high enough to always move the robot
        //return targetSpeed;
    }



    public void drive(double speed, double inches, double timeout){
        //function to move the robot Forwards and Backwards when called in autonomous programs
        drive(speed,speed,inches,inches,timeout);
    }


    public void CCWRotate(double degrees){//Function to facilitate rotating the robot
        double inches = degrees * .06678;
        drive(.3,.3,0,inches,4);

    }

    public void CWRotate(double degrees){//Function to facilitate rotating the robot
        double inches = degrees * .06678;
        drive(.3,.3,inches,0,4);

    }



    public void logPositions(){
        //Prints the motor's encoder positions when called
        opMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                BLD.getCurrentPosition(),
                BRD.getCurrentPosition());
        opMode.telemetry.update();
    }
    private void drive(double Lspeed, double Rspeed,
                       double leftInches, double rightInches,
                       double timeoutS ) {


        //Function to facilitate encoder driving
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active


        // Determine new target position, and pass to motor controller

        int leftstart = BLD.getCurrentPosition();
        int rightstart = BRD.getCurrentPosition();
        newLeftTarget = BLD.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = BRD.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        BLD.setTargetPosition(newLeftTarget);
        BRD.setTargetPosition(newRightTarget);

        //Move motors to position
        BLD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRD.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();
        FRD.setPower(Math.abs(Rspeed));
        FLD.setPower(Math.abs(Lspeed));
        BRD.setPower(Math.abs(Rspeed));
        BLD.setPower(Math.abs(Lspeed));

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (BLD.isBusy() && BRD.isBusy())) {

            // Display it for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    BLD.getCurrentPosition(),
                    BRD.getCurrentPosition());
            opMode.telemetry.addData("lspeed",Lspeed);
            opMode.telemetry.addData("rspeed",Rspeed);
            opMode.telemetry.update();
        }

        // Stop all motion;
        FLD.setPower(0);
        FRD.setPower(0);
        BLD.setPower(0);
        BRD.setPower(0);

        // Turn off RUN_TO_POSITION
        BLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move

    }
}



