
package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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


    private Servo servoPincher; // servos go from 0 to 1 rotates 180 degrees
    double servoPincherInitPosition = 0.5; // doubles store a decimal
    double servoPincherPositionOpen = 0.0;
    double servoPincherPositionClose = 1.0;
    int servoOneDelay = 10;

    //a game pad = a controller

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
        initServoOne();
    }

    private void initDrive() {
        frontLeft = hardwareMap.get(DcMotor.class,"leftFront");
        frontRight = hardwareMap.get(DcMotor.class,"rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class,"rightBack");
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        // backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }
    public void initServoOne() {
        servoPincher = hardwareMap.get(Servo.class, "servoPincher"); // maps the servo
        servoPincher.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoPincher.setPosition(servoPincherInitPosition); // sets the initial position from the variable above.
    }
    public void ServoMovement()
    {
        if(gamepad1.right_bumper)
        {
            servoPincher.setPosition(servoPincherPositionOpen);
        }
        if(gamepad1.left_bumper)
        {
            servoPincher.setPosition(servoPincherPositionClose);
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
        telemetry.addData("Port Number", servoPincher.getConnectionInfo());
        telemetry.addData("Device Name", servoPincher.getDeviceName());
        telemetry.addData("Manufacture", servoPincher.getManufacturer());
        telemetry.addData("Version", servoPincher.getVersion());
        telemetry.addData("Class", servoPincher.getClass());
        telemetry.update();
    }
    public void teleOpControls() {
        driveTrain();
        ServoMovement();
        Lift();
        liftDirection = 0;
    }
}
