
package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


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
 *
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

//ALL THE SERVO STUFF
    private Servo servoPincher; // servos go from 0 to 1 rotates 180 degrees
    double servoPincherInitPosition = 0.0; // doubles store a decimal
    double servoPincherPositionOpen = 0.0;
    double servoPincherPositionClose = 0.3;

    private Servo pincherPivot; // servos go from 0 to 1 rotates 180 degrees
    double pincherPivotInitPosition = 0.0; // doubles store a decimal
    double pincherPivotPositionOutside = 0.0;
    double pincherPivotPositionInside = 0.55;

    private Servo shortArmPivot; // servos go from 0 to 1 rotates 180 degrees
    double shortArmPivotInitPosition = 0.5; // doubles store a decimal
    double shortArmPivotPositionPickUp = 0.0;
    double shortArmPivotPositionPassOff = 1.0;

    private Servo shortArmWrist; // servos go from 0 to 1 rotates 180 degrees
    double shortArmWristInitPosition = 0.0; // doubles store a decimal
    double shortArmWristPositionPickup = 0.0;
    double shortArmWristPositionPassOff = 1.0;

    private Servo trayPivot; // servos go from 0 to 1 rotates 180 degrees
    double trayPivotInitPosition = 0.5; // doubles store a decimal
    double trayPivotPositionPickup = 0.2;
    double trayPivotPositionDropOff = 1.0;

    private CRServo geckoWheel; // servos go from 0 to 1 rotates 180 degrees
    double geckoWheelPower = 1.0; // doubles store a decimal


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
    public void Lift()
    {
        if(gamepad1.y)
        {
            liftDirection = 1;
        }
        else if(gamepad1.a)
        {
            liftDirection = -1;
        }
        else {
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
        frontLeft = hardwareMap.get(DcMotor.class,"leftFront");
        frontRight = hardwareMap.get(DcMotor.class,"rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class,"rightBack");
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

        shortArmWrist = hardwareMap.get(Servo.class, "grabberPivot"); // maps the servo
        shortArmWrist.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        shortArmWrist.setPosition(shortArmWristInitPosition); // sets the initial position from the variable above.

        geckoWheel = hardwareMap.get(CRServo.class, "grabberRotation");
        geckoWheel.setDirection(CRServo.Direction.FORWARD);

        shortArmPivot = hardwareMap.get(Servo.class, "armRotation"); // maps the servo
        shortArmPivot.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        shortArmPivot.setPosition(shortArmPivotInitPosition); // sets the initial position from the variable above.

        trayPivot = hardwareMap.get(Servo.class, "trayPivot"); // maps the servo
        trayPivot.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        trayPivot.setPosition(trayPivotInitPosition); // sets the initial position from the variable above.

    }
    public void ServoMovement()
    {
        if(gamepad1.x)
        {
            trayPivot.setPosition(trayPivotPositionPickup);
        }
        if(gamepad1.b)
        {
            trayPivot.setPosition(trayPivotPositionDropOff);
        }
        if(gamepad1.right_bumper)
        {
            servoPincher.setPosition(servoPincherPositionOpen);
        }
        if(gamepad1.left_bumper)
        {
            servoPincher.setPosition(servoPincherPositionClose);
        }
        if(gamepad1.right_trigger >= 0.5)
        {
            pincherPivot.setPosition(pincherPivotPositionInside);
        }
        if(gamepad1.left_trigger >= 0.5)
        {
            pincherPivot.setPosition(pincherPivotPositionOutside);
        }

        if(gamepad2.right_bumper)
        {

           shortArmPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
            shortArmPivot.setPosition(shortArmPivotPositionPickUp);
        }
        if(gamepad2.left_bumper)
        {
            shortArmPivot.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
            shortArmPivot.setPosition(shortArmPivotPositionPassOff);
        }
        if(gamepad2.right_trigger >= 0.5)
        {
            shortArmWrist.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice

            shortArmWrist.setPosition(shortArmWristPositionPickup);
        }
        if(gamepad2.left_trigger >= 0.5)
        {
            shortArmWrist.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice

            shortArmWrist.setPosition(shortArmWristPositionPassOff);
        }
        if(gamepad2.a)
        {
            geckoWheel.setPower(geckoWheelPower);
        } else if (gamepad2.y) {
            geckoWheel.setPower(-geckoWheelPower);
        }
        else {
            geckoWheel.setPower(0);
        }
    }

    public void driveTrain()
    {
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
        //telemetry.log().clear();
        telemetry.addData("Position", servoPincher.getPosition());
        telemetry.addData("Direction", servoPincher.getDirection());
        telemetry.addData("Controller", servoPincher.getController());

        telemetry.addData("Position", pincherPivot.getPosition());
        telemetry.addData("Direction", pincherPivot.getDirection());
        telemetry.addData("Controller", pincherPivot.getController());

        telemetry.addData("pOWER", geckoWheel.getPower());
        telemetry.addData("Direction", geckoWheel.getDirection());
        telemetry.addData("Port", geckoWheel.getPortNumber());

        telemetry.addData("Position", pincherPivot.getPosition());
        telemetry.addData("Direction", pincherPivot.getDirection());
        telemetry.addData("Controller", pincherPivot.getController());
    }
    public void teleOpControls() {
        driveTrain();
        ServoMovement();
        Lift();
        liftDirection = 0;
    }
}
