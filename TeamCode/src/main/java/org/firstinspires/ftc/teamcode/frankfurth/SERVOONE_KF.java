package org.firstinspires.ftc.teamcode.frankfurth;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Config file
 * Port 00: mortorOne
 * Port 01: mortorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 */
@Disabled
@TeleOp(group = "Frankfurth", name = "Basic TeleOp KF")
public class SERVOONE_KF extends LinearOpMode {
    //Global Variables
    private Servo servoOne; // servos go from 0 to 1 rotates 180 degrees
    double servoOneInitPosition = 0.5; // doubles store a decimal
    double servoOnePositionOne = 0.0;
    double servoOnePositionTwo = 1.0;
    int servoOneDelay = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method init's the hardware motors, servos and sensors
        // single command to run once
        while (!isStarted()) {
            // camera methods that
            servoTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            // method that operate the robot in teleOp
            teleOpControls();
            servoTelemetry();
        }

    }

    public void initHardware() {
        initServoOne();

    }

    public void initServoOne() {
        servoOne = hardwareMap.get(Servo.class, "servoOne"); // maps the servo
        servoOne.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoOne.setPosition(servoOneInitPosition); // sets the initial position from the variable above.
    }

    public void teleOpControls() {
        if (gamepad1.a) {
            servoOne.setPosition(servoOnePositionOne);
        }
        if (gamepad1.b) {
            servoOne.setPosition(servoOnePositionTwo);
        }
        if (gamepad1.right_bumper) {
            servoOneSlower(servoOnePositionOne, servoOnePositionTwo, servoOneDelay);
        }

    }

    public void servoOneSlower(double startPosition, double endPosition, int delay) {
        // maybe for Auto mode.
        double range = ((endPosition - startPosition) * 100);
        //for(local variable; conditional; update variable)
        for (int i = 0; i <= range; i++) {
            servoOne.setPosition(startPosition);
            sleep(delay);
            startPosition = startPosition + 0.01;
        }
    }

    public void servoTelemetry() {
        //telemetry.log().clear();
        telemetry.addData("Position", servoOne.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.addData("Controller", servoOne.getController());
        telemetry.addData("Port Number", servoOne.getConnectionInfo());
        telemetry.addData("Device Name", servoOne.getDeviceName());
        telemetry.addData("Manufacture", servoOne.getManufacturer());
        telemetry.addData("Version", servoOne.getVersion());
        telemetry.addData("Class", servoOne.getClass());
        telemetry.update();
    }

}

