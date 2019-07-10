package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Actual TeleOp", group="Iterative Opmode")
public class Actual_Teleop extends OpMode
{   //declare all variables and name and set servos
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor FRD,FLD,BRD,BLD,RLin,LLin,RLift,LLift;
    public CRServo vexIntakeR, Slock;
    public Servo hangServo;


    float[] hsvValues = new float[3];
    final float values[] = hsvValues;

    @Override
    public void init() {
        //map and name all motors and servos
        telemetry.addData("Status", "Initialized");

        FLD = hardwareMap.get(DcMotor.class, "FLD");
        FRD = hardwareMap.get(DcMotor.class, "FRD");
        BLD = hardwareMap.get(DcMotor.class, "BLD");
        BRD = hardwareMap.get(DcMotor.class, "BRD");
        RLin = hardwareMap.get(DcMotor.class, "RLin");
        LLin = hardwareMap.get(DcMotor.class, "LLin");
        RLift = hardwareMap.get(DcMotor.class, "RLift");
        LLift = hardwareMap.get(DcMotor.class, "LLift");

        vexIntakeR = hardwareMap.get(CRServo.class, "vexIntakeR");
        Slock = hardwareMap.get(CRServo.class, "Slock");
        hangServo = hardwareMap.get(Servo.class, "hangServo");

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset(); }
    boolean pressed = false;
    boolean hangUp = false;
    double lockPower = 0;
    double FLDPower;
    double FRDPower;
    double BLDPower;
    double BRDPower;
    double LinPower;
    double RLiftPower;
    double LLiftPower;
    double LiftPower;

    @Override
    public void loop() {
        //set power variable

        vexIntakeR.setPower(Range.clip(gamepad2.right_stick_y,-.8,.8));


        double rightTrigger1 = gamepad1.right_trigger;
        double leftTrigger1 = gamepad1.left_trigger;
        if(rightTrigger1>0.1){
            lockPower = rightTrigger1;
        }else if (leftTrigger1 >0.1){
            lockPower = -leftTrigger1;
        }else{
            lockPower = 0;
        }

        Slock.setPower(lockPower);

        if(gamepad2.y && hangUp == false){
            hangServo.setPosition(1);
            hangUp = true;
            while (true){
                if (runtime.seconds() >= 2){
                    break;
                }
            }

        }else if(gamepad2.y && hangUp){
            hangServo.setPosition(-1);
            hangUp = false;
            while (true){
                if (runtime.seconds() >= 2){
                    break;
                }
            }
        }

        if (gamepad1.b && gamepad2.b || pressed == true) {
            double out = gamepad2.left_trigger;
            double in = gamepad2.right_trigger;
            LinPower = Range.clip(out-in, -1.0, 1.0);
            RLin.setPower(LinPower);
            LLin.setPower(-LinPower);

            double leftSticky = gamepad1.left_stick_y;
            RLiftPower = Range.clip(leftSticky, -1.0, 1.0);
            LLiftPower = Range.clip(leftSticky, -1.0, 1.0);
            RLift.setPower(-RLiftPower);
            LLift.setPower(LLiftPower);

            double drive = gamepad1.right_stick_x;
            double turn  =  -gamepad1.right_stick_y;
            FLDPower = Range.clip(drive + turn, -0.9, 0.9) ;
            FRDPower = Range.clip(drive - turn, -0.9, 0.9) ;
            BLDPower = Range.clip(drive + turn, -0.9, 0.9) ;
            BRDPower = Range.clip(drive - turn, -0.9, 0.9) ;
            FLD.setPower(-FLDPower);
            FRD.setPower(-FRDPower);
            BLD.setPower(-BLDPower);
            BRD.setPower(-BRDPower);

            pressed = true;

        }else{
            double outLin = gamepad2.left_stick_y;
            LinPower = Range.clip(outLin,  -1.0, 1.0);
            RLin.setPower(LinPower);
            LLin.setPower(-LinPower);
            telemetry.addData("LinPower",LinPower);

            double out = gamepad2.left_trigger;
            double in = gamepad2.right_trigger;
            LiftPower = Range.clip(out-in, -1.0, 1.0);
            RLift.setPower(-LiftPower);
            LLift.setPower(LiftPower);

            double drive = gamepad1.right_stick_x;
            double turn  =  -gamepad1.right_stick_y;
            FLDPower = Range.clip(drive + turn, -0.9, 0.9) ;
            FRDPower = Range.clip(drive - turn, -0.9, 0.9) ;
            BLDPower = Range.clip(drive + turn, -0.9, 0.9) ;
            BRDPower = Range.clip(drive - turn, -0.9, 0.9) ;
            FLD.setPower(-FLDPower);
            FRD.setPower(-FRDPower);
            BLD.setPower(-BLDPower);
            BRD.setPower(-BRDPower);

        }
    }
    @Override
    public void stop() {
    }
}
