package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Config File
 * Port 00: motorFour]\\\nv'
 * MBR
 * Port 01: motorThree MBL
 * Port 02: motorTwo MFR
 * Port 03: motorOne MFL
 * <p>
 * Port 00: Servo grabber
 * Port 01: Servo pivot
 * Port 02: Servo extension
 */
//@Disabled
@TeleOp(group = "Team5233", name = "BasicTeleopV1.1")
public class BASICTELEOPV1 extends LinearOpMode {
    double liftDirection = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;

    float DEADBAND = 0.05f;

    double grabberInitPosition = 0.0; // doubles store a decimal
    double grabberPositionOpen = 0.0;
    double grabberPositionClose = 0.3;
    double pivotInitPosition = 0.0;   // doubles store a decimal
    double pivotPositionPickUp = 1.0; // TODO: check this and next value
    double pivotPositionPassOff = 0.0;
    double extensionInitPosition = 0.0; // doubles store a decimal
    double extensionExtended = 0.0;     // TODO: set correct numbers for extension
    double extensionRetracted = 1.0;    // TODO: set correct number for retracted
    // ALL THE SERVO STUFF

    private Servo grabber;   // servos go from 0 to 1 rotates 180 degrees
    private Servo pivot;     // servos go from 0 to 1 rotates 180 degrees
    private Servo extension; // servos go from 0 to 1 rotates 180 degrees

    private int liftPosition;

    @Override public void runOpMode() throws InterruptedException
    {
        initHardware();
        while (!isStarted())
        {
        }
        waitForStart();
        while (opModeIsActive())
        {
            servoTelemetry();
            teleOpControls();
            // SetLift(liftPosition);
        }
    }

    /*
    public boolean SetLift(int LiftPosition) {
        //This function might be buggy
        float padding = 0.5f; //this value might need to change
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setTargetPosition(-(LiftPosition));
        liftLeft.setTargetPosition(LiftPosition);

        if (liftRight.getCurrentPosition() < LiftPosition + padding){
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        return false;
                }
        else if(liftRight.getCurrentPosition() > LiftPosition - padding){
            liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
            liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
            return false;
        }
        else{
            liftRight.setPower(0.0);
            liftLeft.setPower(0.0);
            return true;
        }

    }
*/
    //
    public void Lift()
    {
        if (gamepad1.y)
        {
            liftDirection = 1;

        }
        else if (gamepad1.a)
        {
            liftDirection = -1;
        }
        liftRight.setPower(1);
        liftLeft.setPower(1);
    }

    public void initHardware()
    {
        initDrive();
        initServos();
    }

    private void initDrive()
    {
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

        // back left reverse os up for debate because we changed hardware and it stopped working
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initServos()
    {
        grabber = hardwareMap.get(Servo.class, "grabber"); // maps the servo
        grabber.setDirection(
                Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        grabber.setPosition(
                grabberInitPosition); // sets the initial position from the variable above.

        pivot = hardwareMap.get(Servo.class, "pivot"); // maps the servo
        pivot.setDirection(
                Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        pivot.setPosition(pivotInitPosition); // sets the initial position from the variable above.

        extension = hardwareMap.get(Servo.class, "extension"); // maps the servo
        extension.setDirection(
                Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        extension.setPosition(
                extensionInitPosition); // sets the initial position from the variable above.
    }

    private float falloff(float input)
    {
        if (Math.abs(input) < DEADBAND)
        {
            input = 0;
        }
        // Curve for smoothing drivetrain
        return input * input * input;
    }

    public void ServoMovement()
    {
        grabber();
        pivot();
        extendPincher();
    }

    private void extendPincher()
    {
        if (gamepad2.b)
        {
            // shortArmWrist.setDirection(Servo.Direction.FORWARD); // sets the direction of
            // rotation - optional but good practice
            //     shortArmPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of
            //     rotation - optional but good practice
            extension.setPosition(extensionRetracted);
        }
        if (gamepad2.x)
        {
            // shortArmWrist.setDirection(Servo.Direction.REVERSE); // sets the direction of
            // rotation - optional but good practice
            //  shortArmWrist.setPosition(shortArmWristPositionPassOff);
            //    shortArmPivot.setDirection(Servo.Direction.FORWARD); // sets the direction of
            //    rotation - optional but good practice
            extension.setPosition(extensionExtended);
        }
    }

    private void pivot()
    {
        if (gamepad2.right_trigger >= 0.5)
        {
            pivot.setPosition(pivotPositionPickUp);
        }
        if (gamepad2.left_trigger >= 0.5)
        {
            pivot.setPosition(pivotPositionPassOff);
        }
    }

    private void grabber()
    {
        if (gamepad1.right_bumper)
        {
            grabber.setPosition(grabberPositionOpen);
        }
        if (gamepad1.left_bumper)
        {
            grabber.setPosition(grabberPositionClose);
        }
    }

    private void grab()
    {
        /*
        if (gamepad1.y)
        {
            //TODO: set this to optimal height for scoring specimens
            SetLift(1);
            if (SetLift(1) == true)
            {
                extension.setPosition(extensionExtended);
                grabber.setPosition(grabberPositionOpen);
            }
        }
        else if(gamepad1.a){
            extension.setPosition(extensionExtended);
            SetLift(96);

        }
        else
        {
            grabber.setPosition(grabberPositionClose);
        }
    }
         */
    }

    public void driveTrain()
    {
        double lx = falloff(gamepad1.left_stick_x);
        double ly = falloff(gamepad1.left_stick_y);
        double rx = falloff(gamepad1.right_stick_x);
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        frontLeft.setPower((ly + lx + rx) / max);
        frontRight.setPower((ly - lx - rx) / max);
        backLeft.setPower((ly - lx + rx) / max);
        backRight.setPower((ly + lx - rx) / max);
    }

    public void servoTelemetry()
    {
        //        telemetry.addData("Pincher Position", grabber.getPosition());
        //        telemetry.addData("Pincher Direction", grabber.getDirection());
        //        telemetry.addData("Pincher Controller", grabber.getController());
        //
        //        telemetry.addData("Pivot Position", pivot.getPosition());
        //        telemetry.addData("Pivot Direction", pivot.getDirection());
        //        telemetry.addData("Pivot Controller", pivot.getController());

        //        telemetry.addData("Extension Position", extension.getPosition());
        //        telemetry.addData("Extension Direction",extension.getDirection());
        //        telemetry.addData("Extension Controller",extension.getController());

        telemetry.addData("Left lift position", liftLeft.getCurrentPosition());
        telemetry.addData("Right lift position", liftRight.getCurrentPosition());
        telemetry.update();
    }

    public void teleOpControls()
    {
        servoTelemetry();
        driveTrain();
        ServoMovement();
        Lift();
        grab();
        // liftDirection = 0;
    }
}
