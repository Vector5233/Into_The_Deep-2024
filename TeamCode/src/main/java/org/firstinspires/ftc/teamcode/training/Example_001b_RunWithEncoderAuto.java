package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Robot Config
 * Control Hub
 *
 * Expansion Hub
 */

//@Disabled
@Autonomous (group="secondary", name="Run To Pos Enc Auto")

public class Example_001b_RunWithEncoderAuto extends LinearOpMode {
    //global variables
    private DcMotor motorThree;
    double speed = 1;
    double motorThreeZeroPower = 0.0;
    double motorThreePower = Math.abs(speed);
    int motorThreePositionOne = 0;
    int motorThreePositionTwo = 1000;
    int motorThreePositionThree = -200;
    // servo variables
    private Servo servoOne; // servos go from 0 to 1 rotates 180 degrees
    double servoOneInitPosition = 0.5; // doubles store a decimal
    double servoOnePositionOne = 0.0;
    double servoOnePositionTwo = 1.0;
    int servoOneDelay = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        //define runOpMode variables
        //load in inits
        inits();
        waitForStart();

        while (!isStarted() && !isStopRequested()) {
            motorTelemetry();
            servoTelemetry();
        }
        // main loop
//        while (opModeIsActive()) {
//            // commands to run
//
//            motorTelemetry();
//
//
//
//
//           // sleep(5000);
//        }
        // individual commands go here.
        
        if(opModeIsActive()) {
            runMotorThreeToPosition(motorThreePositionTwo);
            while (motorThree.isBusy() && opModeIsActive()) {
                sleep(50);
            }
            if(motorThree.getCurrentPosition() >= 900){
                servoOne.setPosition(servoOnePositionOne);
            }
            servoTelemetry();

            runMotorThreeToPosition(motorThreePositionThree);
            while (motorThree.isBusy() && opModeIsActive()) {
                sleep(50);
            }
            //stopMotors();
            motorTelemetry();

                sleep(1000);
           if(motorThree.getCurrentPosition()<=200) {
               servoOne.setPosition(servoOnePositionTwo); //
               sleep(500);
           }
            servoTelemetry();
        }
        sleep(2500);


        requestOpModeStop();
    }

// function stack
    public void motorTelemetry() {
        telemetry.addData("motorThree", "Encoder: %2d, Power: %2f", motorThree.getCurrentPosition(), motorThree.getPower());
        telemetry.update();
        //functions here
    }

    public void servoTelemetry(){
        telemetry.addData("Position", servoOne.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.update();
    }
    public void inits(){
        initMotorThree();
        initServoOne();
    }



    public void initMotorThree() {
        motorThree = hardwareMap.get(DcMotor.class, "motorThree");
        motorThree.setDirection(DcMotorSimple.Direction.FORWARD);
        motorThree.setPower(motorThreeZeroPower);
        motorThree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BRAKE stop as fast as possible
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void runMotorThreeToPosition(int position) {
        motorThree.setTargetPosition(position);
        motorThree.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorThree.setPower(motorThreePower);
        while (motorThree.isBusy()) {
            motorTelemetry();
        }
        motorThree.setPower(motorThreeZeroPower);// Optional /
    }

    public void resetEncoders(){
        stopMotors();
        stopAndResetEncoders();
        resetMode();
    }
    public void stopMotors(){
        motorThree.setPower(motorThreeZeroPower);
    }
    public void stopAndResetEncoders(){
        motorThree.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetMode(){
        motorThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void initServoOne(){
        servoOne = hardwareMap.get(Servo.class, "servoOne"); // maps the servo
        servoOne.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoOne.setPosition(servoOneInitPosition); // sets the initial position from the variable above.
    }



}
