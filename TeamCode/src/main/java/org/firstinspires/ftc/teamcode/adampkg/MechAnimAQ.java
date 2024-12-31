package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: fix config file

/**
 * Config File
 * Port 00: motorOne MFL
 * Port 01: motorTwo MFR
 * Port 02: motorThree MBL
 * Port 03: motorFour MBR
 * <p>
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 */
//@Disabled
@TeleOp(group = "Qureshi", name = "BasicTeleop")
public class MechAnimAQ extends LinearOpMode {
    double liftDirection = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;
    double servoPincherInitPosition = 0.0; // doubles store a decimal
    double servoPincherPositionOpen = 0.0;
    double servoPincherPositionClose = 0.3;
    double pincherPivotInitPosition = 0.0; // doubles store a decimal
    double pincherPivotPositionPickUp = 1.0; //TODO: check this and next value
    double pincherPivotPositionPassOff = 0.1;
    double extensionInitPosition = 0.5; // doubles store a decimal
    double extensionExtended = 0.0; //TODO: set correct numbers for extension
    double extensionRetracted = 0.0; //TODO: set correct number for retracted
    //ALL THE SERVO STUFF
    private Servo servoPincher; // servos go from 0 to 1 rotates 180 degrees
    private Servo pincherPivot; // servos go from 0 to 1 rotates 180 degrees
    private Servo extension; // servos go from 0 to 1 rotates 180 degrees

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            servoTelemetry();
            teleOpControls();
        }

    }

    public void Lift() {
        if (gamepad1.y) {
            liftDirection = 1;
        } else if (gamepad1.a) {
            liftDirection = -1;
        } else {
            liftDirection = 0;
        }
        liftRight.setPower(liftDirection);
        liftLeft.setPower(liftDirection);

        //return liftDirection;
    }

    public void initHardware() {
        initDrive();
        initServos();
    }

    private void initDrive() {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        //back left reverse os up for debate because we changed hardware and it stopped working
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initServos() {
        servoPincher = hardwareMap.get(Servo.class, "servoPincher"); // maps the servo
        servoPincher.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoPincher.setPosition(servoPincherInitPosition); // sets the initial position from the variable above.

        pincherPivot = hardwareMap.get(Servo.class, "pincherPivot"); // maps the servo
        pincherPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        pincherPivot.setPosition(pincherPivotInitPosition); // sets the initial position from the variable above.


        extension = hardwareMap.get(Servo.class, "extension"); // maps the servo
        extension.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        extension.setPosition(extensionInitPosition); // sets the initial position from the variable above.
    }

    public void ServoMovement() {
        ServoPincher();
        PincherPivot();
        extendPincher();
    }

    private void extendPincher() {
        if (gamepad2.b) {
            //shortArmWrist.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
            //    shortArmPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
            extension.setPosition(extensionExtended);
        }
        if (gamepad2.x) {
            // shortArmWrist.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
            //  shortArmWrist.setPosition(shortArmWristPositionPassOff);
            //    shortArmPivot.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
            extension.setPosition(extensionRetracted);
        }
    }

    private void PincherPivot() {
        if (gamepad1.right_trigger >= 0.5) {
            pincherPivot.setPosition(pincherPivotPositionPickUp);
        }
        if (gamepad1.left_trigger >= 0.5) {
            pincherPivot.setPosition(pincherPivotPositionPassOff);
        }
    }

    private void ServoPincher() {
        if (gamepad1.right_bumper) {
            servoPincher.setPosition(servoPincherPositionOpen);
        }
        if (gamepad1.left_bumper) {
            servoPincher.setPosition(servoPincherPositionClose);
        }
    }

    public void driveTrain() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        frontLeft.setPower((ly + lx + rx) / max);
        frontRight.setPower((ly - lx - rx) / max);
        backLeft.setPower((ly - lx + rx) / max);
        backRight.setPower((ly + lx - rx) / max);
    }

    public void servoTelemetry() {
        //telemetry.log().clear()
        telemetry.addData("Pincher Position", servoPincher.getPosition());
        telemetry.addData("Pincher Direction", servoPincher.getDirection());
        telemetry.addData("Pincher Controller", servoPincher.getController());

        telemetry.addData("Pivot Position", pincherPivot.getPosition());
        telemetry.addData("Pivot Direction", pincherPivot.getDirection());
        telemetry.addData("Pivot Controller", pincherPivot.getController());


        telemetry.addData("Extension Position", extension.getPosition());
        telemetry.addData("Extension Direction",extension.getDirection());
        telemetry.addData("Extension Controller",extension.getController());
    }
    public void teleOpControls() {
        driveTrain();
        ServoMovement();
        Lift();
        liftDirection = 0;
    }
}
