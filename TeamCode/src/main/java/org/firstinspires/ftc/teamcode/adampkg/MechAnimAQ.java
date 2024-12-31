package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//TODO: fix config file

/**
 * Config File
 * Port 00: motorOne MFL
 * Port 01: motorTwo MFR
 * Port 02: motorThree MBL
 * Port 03: motorFour MBR
 *
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

    private int liftPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            servoTelemetry();
            teleOpControls();
            SetLift(liftPosition);
        }

    }
    public boolean SetLift(int LiftPosition) {
        //TODO: tune this function
        //This function might be buggy
        float padding = 0.5f; //this value might need to change
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setTargetPosition(LiftPosition);
        liftLeft.setTargetPosition(LiftPosition);
        return !(liftLeft.isBusy()|| liftRight.isBusy());
//        if (liftRight.getCurrentPosition() < LiftPosition + padding){
//        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        return false;
//                }
//        else if(liftRight.getCurrentPosition() > LiftPosition - padding){
//            liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            return false;
//        }
//        else{
//            liftRight.setPower(0.0);
//            liftLeft.setPower(0.0);
//            return true;
//        }
    }
    public void Lift() {
        if (gamepad1.y) {
            liftPosition = 1;
        } else if (gamepad1.a) {
            liftPosition = 0;
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
        liftRight.setDirection(DcMotor.Direction.FORWARD);
        liftLeft.setPower(0.0);
        liftRight.setPower(0.0);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    private void grab(){
        if (gamepad1.right_trigger >= 0) {
            if(gamepad1.left_trigger >= 0) {
                //TODO: set this to optimal height for scoring specimens
                if(SetLift(1) == true){
                    extension.setPosition(extensionExtended);
                }

            }
            else{
                servoPincher.setPosition(servoPincherPositionClose);
            }
        } else {
            //TODO: set this to optimal height for pickup from wall
            if(SetLift(0) == true){
                extension.setPosition(extensionExtended);
                servoPincher.setPosition(servoPincherPositionOpen);

            }

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

        telemetry.addData("Left lift position",liftLeft.getCurrentPosition());
        telemetry.addData("Right lift position",liftRight.getCurrentPosition());
    }
    public void teleOpControls() {
        driveTrain();
        ServoMovement();
        Lift();
        grab();
        liftDirection = 0;
    }
}
