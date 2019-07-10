package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class DriveUtilities {

    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    //Declare Motors and servos
    DcMotor FRD,FLD,BRD,BLD,RLin,LLin,RLift,LLift;
    Servo hangServo;
    CRServo Slock, vexIntakeR;

    LinearOpMode opMode;
    //declare constants
    private static final double     ROBOT_DIAMETER          = 16;
    private static final double     DIST_BETWEEN_EDGES      = 12;
    private static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;
    private static final double     ACCELERATION_GRADIENT   = 1.0/600;
    private static final double     DECELERATION_GRADIENT   = 1.0/2000;//rate of acceleation and decelleration of robot
    private static final double     MINIMUM_POWER           = 0.1;
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 3.77 ;
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public DriveUtilities(HardwareMap hardwareMap, LinearOpMode opMode) {
        //Initialize motors and variables
        FLD = hardwareMap.get(DcMotor.class, "FLD");
        FRD = hardwareMap.get(DcMotor.class, "FRD");
        BLD = hardwareMap.get(DcMotor.class, "BLD");
        BRD = hardwareMap.get(DcMotor.class, "BRD");
        LLin = hardwareMap.get(DcMotor.class, "LLin");
        RLin= hardwareMap.get(DcMotor.class, "RLin");
        RLift = hardwareMap.get(DcMotor.class, "RLift");
        LLift = hardwareMap.get(DcMotor.class, "LLift");

        hangServo = hardwareMap.get(Servo.class, "hangServo");
        Slock = hardwareMap.get(CRServo.class, "Slock");
        vexIntakeR = hardwareMap.get(CRServo.class, "vexIntakeR");


       // sensorRange = hardwareMap.get(DistanceSensor.class, "dst");


        //Set encoder modes
        //FLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //FRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //FLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        return Math.sqrt(Math.max(MINIMUM_POWER, clippedPower));  //Ensure power is high enough to always move the robot
        //return targetSpeed sqrt for calculus purposes;
    }
    public void CCWRotate(double degrees){//Function to facilitate rotating the robot
         double inches = (15.5 * 2 * Math.PI) * (degrees / 360);
        driveM(-.3,.3,-inches,inches,10);

    }

    public void CWRotate(double degrees){//Function to facilitate rotating the robot
        double inches = (15.5 * 2 * Math.PI) * (degrees / 360);
        driveM(.3,-.3,inches,-inches,10);

    }


    public void drive(double speed, double inches, double timeout){
        //function to move the robot Forwards and Backwards when called in autonomous programs
        driveM(speed,speed,inches,inches,timeout);
    }
  

    //Function to move the robot in an arcing motion
    
    


    public void logPositions(){
        //Prints the motor's encoder positions when called
        opMode.telemetry.addData("Path0", "Starting at %7d :%7d",
                BLD.getCurrentPosition(),
                BRD.getCurrentPosition());
        opMode.telemetry.update();
    }

    /**
     *
     * @param leftPower A positive real value between 0 and 1 that sets the speed of the motor
     * @param rightPower A positive real value between 0 and 1 that sets the speed of the motor
     * @param leftInches
     * @param rightInches
     * @param timeoutS
     */
    private void driveM(double leftPower, double rightPower,
                       double leftInches, double rightInches,
                       double timeoutS) {


        int newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
        int newRightTarget = (int) (rightInches * COUNTS_PER_INCH);

        BRD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //BRD.setDirection(DcMotor.Direction.FORWARD);
        //FLD.setDirection(DcMotor.Direction.FORWARD);
        //BLD.setDirection(DcMotor.Direction.FORWARD);
        //FRD.setDirection(DcMotor.Direction.FORWARD);

        runtime.reset();

        opMode.telemetry.addData("Right Encoder", BRD.getCurrentPosition());
        opMode.telemetry.addData("Left Encoder", BLD.getCurrentPosition());
        opMode.telemetry.update();
        opMode.sleep(1000);

        //double rightTargetPower = Integer.signum(newRightTarget) * rightPower;
        //double leftTargetPower = Integer.signum(newLeftTarget) * leftPower;
        double rightTargetPower = rightPower;
        double leftTargetPower = leftPower;

        BRD.setPower(rightTargetPower);
        FRD.setPower(rightTargetPower);
        BLD.setPower(-leftTargetPower);
        FLD.setPower(-leftTargetPower);

        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeoutS)) {

            // Have we reached our targets?

            //BRD.setPower(getPower(rightTargetPower,BRD.getCurrentPosition(),newRightTarget,0));
            //FRD.setPower(getPower(rightTargetPower,BRD.getCurrentPosition(),newRightTarget,0));
            //BLD.setPower(getPower(leftTargetPower,BLD.getCurrentPosition(),newLeftTarget,0));
            //FLD.setPower(getPower(leftTargetPower,BLD.getCurrentPosition(),newLeftTarget,0));


            int rightDistanceRemaining = Math.abs(newRightTarget) - Math.abs(BRD.getCurrentPosition());
            int leftDistanceRemaining = Math.abs(newLeftTarget) - Math.abs(BLD.getCurrentPosition());

            if (rightDistanceRemaining <=0 && leftDistanceRemaining <=0) {
                break;
            }

        /*
            BRD.setPower(getPower(Math.abs(Rspeed), BRD.getCurrentPosition(), newRightTarget,rightstart));
            FRD.setPower(getPower((Rspeed), BRD.getCurrentPosition(), newRightTarget,rightstart));
            BLD.setPower(getPower(Math.abs(Lspeed), BLD.getCurrentPosition(), newLeftTarget, leftstart));
            FLD.setPower(-1*(getPower((Lspeed), BLD.getCurrentPosition(), newLeftTarget, leftstart)));

            opMode.telemetry.addData("RmotorPower " ,getPower(Math.abs(rightPower), BLD.getCurrentPosition(), newRightTarget, rightStart));
            opMode.telemetry.addData("LmotorPower ", getPower(Math.abs(leftPower), BRD.getCurrentPosition(), newLeftTarget,leftStart));
*/


            // Display it for the driver.
            opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                    BRD.getCurrentPosition(),
                    BLD.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion;
        BRD.setPower(0);
        BLD.setPower(0);
        FRD.setPower(0);
        FLD.setPower(0);

        //  sleep(250);   // optional pause after each move

    }
}



