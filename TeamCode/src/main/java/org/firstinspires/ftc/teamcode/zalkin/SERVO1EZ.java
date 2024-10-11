package org.firstinspires.ftc.teamcode.zalkin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Configuration file
 * Port 00: motorOne
 * Port 01: motorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo contServoOne
 *
 * @noinspection SpellCheckingInspectionBASICTELEOPSTRUCTURE
 */
//@Disabled
@TeleOp(group = "Zalkin", name = "BasicTeleop")
public class SERVO1EZ extends LinearOpMode {
    double servoOneInitPosition = 0.5;
    double servoOnePositionOne = 0.0;
    double servoOnePositionTwo = 1.0;
    int servoOneDelay = 10;
    private Servo servoOne;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method initiates hardware components
        //runs once
        while (!isStarted()) {
            // camera methods that
            servoTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            //method that operates the robot in teleop
            servoTelemetry();
        }
    }

    public void initHardware() {
        initServoOne();
        servoTelemetry();
    }

    public void initServoOne() {
        servoOne = hardwareMap.get(Servo.class, "servoOne"); //maps the servo
        servoOne.setDirection(Servo.Direction.FORWARD); //sets the direction of rotation
        servoOne.setPosition(servoOneInitPosition); //sets the initial position from the variable above
    }

    public void teleOpControls() {
        if (gamepad1.a) {
            servoOne.setPosition(servoOnePositionOne);
        }
        if (gamepad1.b) {
            servoOne.setPosition(servoOnePositionTwo);
        }
        if (gamepad1.right_bumper) {
            //servoOneSlower(servoOnePositionOne, servoOnePositionTwo, servoOneDelay);
        }
    }

    public void servoOneSlower(double startPosition, double endPosition, int delay) {
        // maybe for Auto mode
        double range = ((endPosition - startPosition) * 100);
        //for(local variable; conditional; update variable
        for (int i = 0; i <= range; i++) {
            servoOne.setPosition(startPosition);
            sleep(delay);
            startPosition += 0.01;
        }
    }

    public void servoTelemetry() {
        telemetry.addData("position", servoOne.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.addData("Controller", servoOne.getController());
        telemetry.addData("Port Number", servoOne.getConnectionInfo());
        telemetry.addData("Device Name", servoOne.getDeviceName());
        telemetry.addData("Manufacture", servoOne.getManufacturer());
        telemetry.addData("version", servoOne.getVersion());
        telemetry.addData("Class", servoOne.getClass());
        telemetry.update();
    }
}
