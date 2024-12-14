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
    private DcMotor motorThree;
    double speed = 1;
    double motorThreeZeroPower = 0.0;
    double motorThreePower = Math.abs(speed);
    int motorThreePositionOne = 0;
    int motorThreePositionTwo = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        //define runOpMode variables
//        int motorThreePositionOne = 0;
//        int motorThreePositionTwo = 1000;


        waitForStart();
        while (!opModeIsActive()) {
            //load in inits
            motorTelemetry();
            initMotorThree();
        }

        while (opModeIsActive()) {
            // commands to run
            initMotorThree();
            motorTelemetry();
            //runMotorThreeToPosition(motorThreePositionOne);
           while(){
               runMotorThreeToPosition(motorThreePositionTwo);
           }
            motorThree.setPower(0);
            //sleep(5000);
        }

        //sleep(500);
        requestOpModeStop();

    }
    public void motorTelemetry() {
        telemetry.addData("Note", "Tap y to reset Encoders");
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
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        motorThree.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
