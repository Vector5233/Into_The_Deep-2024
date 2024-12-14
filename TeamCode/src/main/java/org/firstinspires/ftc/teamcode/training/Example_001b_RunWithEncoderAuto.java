package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    //private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorThree;
    double speed = 1;
    double motorThreeZeroPower = 0.0;
    double motorThreePower = Math.abs(speed);
    int motorThreePositionOne = 0;
    int motorThreePositionTwo = 1000;
    int motorThreePositionThree = -200;
    double timeOutS;
    @Override
    public void runOpMode() throws InterruptedException {
        //define runOpMode variables
        //load in inits
        initMotorThree();

        waitForStart();
        while (!isStarted() && !isStopRequested()) {
            motorTelemetry();

        }
        // main loop
        while (opModeIsActive()) {
            // commands to run

            motorTelemetry();
            runMotorThreeToPosition(motorThreePositionTwo);
            sleep(500);
            runMotorThreeToPosition(motorThreePositionThree);

            //stopAndResetEncoders();
           // sleep(5000);
        }
        stopMotors();
        // individual commands go here.


        //sleep(500);
        requestOpModeStop();
    }


    public void motorTelemetry() {
        telemetry.addData("motorThree", "Encoder: %2d, Power: %2f", motorThree.getCurrentPosition(), motorThree.getPower());
        telemetry.update();
        //functions here
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

//    public void raiseLift(int position){
//        motorThree.setTargetPosition(motorThreePositionTwo);
//        motorThree.setPower(speed);
//        while (opModeIsActive() && motorThree.isBusy()){
//            sleep(1);
//        }
//
//    }
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
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
