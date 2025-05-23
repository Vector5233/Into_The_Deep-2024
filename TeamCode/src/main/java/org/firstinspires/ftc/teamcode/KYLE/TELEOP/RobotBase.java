package org.firstinspires.ftc.teamcode.KYLE.TELEOP;

//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class RobotBase {
    double liftDirection = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;
    //ALL THE SERVO STUFF
    Servo servoPincher; // servos go from 0 to 1 rotates 180 degrees
    double servoPincherInitPosition = 0.08; // doubles store a decimal
    double servoPincherPositionClosed = 0.08;
    double servoPincherPositionOpen = 0.3;
    Servo pincherPivot; // servos go from 0 to 1 rotates 180 degrees
    double pincherPivotInitPosition = 0.0; // doubles store a decimal
    double pincherPivotDown = 0.7;
    double pincherPivotUp = 0.0;
    Servo extension; // servos go from 0 to 1 rotates 180 degrees
    double extensionInitPosition = 1.0; // doubles store a decimal
    double extensionExtended = 0.0;
    double extensionRetracted = 1.0;


    public void initServos(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        servoPincher = hardwareMap.get(Servo.class, "grabber"); // maps the servo
        servoPincher.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoPincher.setPosition(servoPincherInitPosition); // sets the initial position from the variable above.

        pincherPivot = hardwareMap.get(Servo.class, "pivot"); // maps the servo
        pincherPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        pincherPivot.setPosition(pincherPivotInitPosition); // sets the initial position from the variable above.
/*
        shortArmWrist = hardwareMap.get(Servo.class, "grabberPivot"); // maps the servo
        shortArmWrist.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        shortArmWrist.setPosition(shortArmWristInitPosition); // sets the initial position from the variable above.

        geckoWheel = hardwareMap.get(CRServo.class, "grabberRotation");
        geckoWheel.setDirection(CRServo.Direction.FORWARD);

        trayPivot = hardwareMap.get(Servo.class, "trayPivot"); // maps the servo
        trayPivot.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        trayPivot.setPosition(trayPivotInitPosition); // sets the initial position from the variable above.
  */
        extension = hardwareMap.get(Servo.class, "extension"); // maps the servo
        extension.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        extension.setPosition(extensionInitPosition); // sets the initial position from the variable above.
    }
    public void initDrive(HardwareMap hardwareMap) {
        frontLeft = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class, "leftBack");
        backRight = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class, "rightBack");
        liftRight = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        //back left reverse os up for debate because we changed hardware and it stopped working
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }


    public void OpenPincher()
    {
        servoPincher.setPosition(servoPincherPositionOpen);
    }
    void Extension(Gamepad gamepad2) {
        if(gamepad2.b)
        {
            extension.setPosition(extensionExtended);
        }
        if(gamepad2.x)
        {
            extension.setPosition(extensionRetracted);
        }
    }
}
