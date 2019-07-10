package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class WorldLiftUtilities {

    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    //Declare Motors and servos
    DcMotor FRD,FLD,BRD,BLD,RLin,LLin, RLift,LLift;
    LinearOpMode opMode;
    //declare constants
    private static final double     COUNTS_PER_MOTOR_REV    = 288 ;
    private static final double     ACCELERATION_GRADIENT   = 1.0/200;
    private static final double     DECELERATION_GRADIENT   = 1.0/2000;//rate of acceleation and decelleration of robot
    private static final double     MINIMUM_POWER           = 0.1;
    private static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    private static final double     ROTATION_PER_TICK         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION);

    public WorldLiftUtilities(HardwareMap hardwareMap, LinearOpMode opMode) {
        //Initialize motors and variables
        FLD = hardwareMap.get(DcMotor.class, "FLD");
        FRD = hardwareMap.get(DcMotor.class, "FRD");
        BLD = hardwareMap.get(DcMotor.class, "BLD");
        BRD = hardwareMap.get(DcMotor.class, "BRD");
        RLin = hardwareMap.get(DcMotor.class, "RLin");
        LLin = hardwareMap.get(DcMotor.class, "LLin");
        RLift = hardwareMap.get(DcMotor.class, "RLift");
        LLift = hardwareMap.get(DcMotor.class, "LLift");

        RLift.setDirection(DcMotor.Direction.FORWARD);
        LLift.setDirection(DcMotor.Direction.FORWARD);


        //Set encoder modes
        RLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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



    public void lift(double speed, double rotations, double timeout){
        //function to move the robot Forwards and Backwards when called in autonomous programs
        drive(speed,rotations,timeout);
    }



    public void logPositions(){
        //Prints the motor's encoder positions when called
        opMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                LLift.getCurrentPosition(),
                RLift.getCurrentPosition());
        opMode.telemetry.update();
    }
    private void drive( double speed,
                        double rotations,
                       double timeoutS ) {
        //Function to facilitate encoder driving
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active


        // Determine new target position, and pass to motor controller

        newLeftTarget = LLift.getCurrentPosition() + (int) (rotations * ROTATION_PER_TICK);
        newRightTarget = RLift.getCurrentPosition() + (int) (rotations * ROTATION_PER_TICK);
        LLift.setTargetPosition(newLeftTarget);
        RLift.setTargetPosition(newRightTarget);

        //Move motors to position
        LLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        runtime.reset();
        LLift.setPower(Math.abs(speed));
        RLift.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (LLift.isBusy() && RLift.isBusy())) {


            //m1.setPower(getPower(Math.abs(Lspeed), m1.getCurrentPosition(), newLeftTarget,leftstart));
            //m2.setPower(getPower(Math.abs(Rspeed), m2.getCurrentPosition(), newRightTarget, rightstart));

            //opMode.telemetry.addData("RmotorPower " ,getPower(Math.abs(Rspeed), m2.getCurrentPosition(), newRightTarget, rightstart));
            //opMode.telemetry.addData("LmotorPower ", getPower(Math.abs(Lspeed), m1.getCurrentPosition(), newLeftTarget,leftstart));



            // Display it for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    LLift.getCurrentPosition(),
                    RLift.getCurrentPosition());
            opMode.telemetry.addData("lspeed",speed);
            opMode.telemetry.addData("rspeed",speed);
            opMode.telemetry.update();
        }

        // Stop all motion;
        LLift.setPower(0);
        RLift.setPower(0);

        // Turn off RUN_TO_POSITION
        LLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //  sleep(250);   // optional pause after each move

    }
}



