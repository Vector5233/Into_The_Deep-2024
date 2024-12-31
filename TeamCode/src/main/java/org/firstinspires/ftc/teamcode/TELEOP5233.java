
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Config File
 * Control Hub
 * Port 00: frontLeft 
 * Port 01: frontRight
 * Port 02: backLeft
 * Port 03: backRight
 * Servos
 *  * Port 00: Servo Extension
 *  * Port 01: Servo Pivot
 *  * Port 02: Servo Grabber
 *  
 * Expansion Hub
 * port 00: liftRight;
 * port 01: liftLeft;
 * Servos
 *
 */
//@Disabled
@TeleOp(group = "TELEOP", name = "5233-Teleop.V1")
public class TELEOP5233 extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;
    double liftDirection = 0;

    double liftPower=0.8;//ALL THE SERVO STUFF
    private Servo Grabber; // servos go from 0 to 1 rotates 180 degrees
    double  GrabberInit = 0.0; // doubles store a decimal
    double GrabberOpen = 0.0;
    double GrabberClosed = 0.3;
    
    private Servo Pivot; // servos go from 0 to 1 rotates 180 degrees
    double PivotInitPosition = 0.0; // doubles store a decimal
    double PivotUp = 0.0;
    double PivotDown = 0.55;

    private Servo Extension; // servos go from 0 to 1 rotates 180 degrees
    double ExtensionInitPosition = 0.5; // doubles store a decimal
    double ExtensionOut = 1.0;
    double ExtensionIn = 0.1;

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
        // Grabber Servo 
        Grabber = hardwareMap.get(Servo.class, "Grabber"); // maps the servo
        Grabber.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        Grabber.setPosition(GrabberInit); // sets the initial position from the variable above.
        //Pivot Servo
        Pivot = hardwareMap.get(Servo.class, "pincherPivot"); // maps the servo
        Pivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        Pivot.setPosition(PivotInitPosition); // sets the initial position from the variable above.
        //Extension Servo
        Extension = hardwareMap.get(Servo.class, "armRotation"); // maps the servo
        Extension.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        Extension.setPosition(ExtensionInitPosition); // sets the initial position from the variable above.
    }
    // calls to the method stack methods
    public void teleOpControls() {
        driveTrain();
        Lift();
        GrabberControls();
        Pivot();
        Extension();
        //liftDirection = 0;// why this?
    }
    // Method Stack below

    // This method controls the opening and closing of the grabber using the right and left bumpers.
    // This should rotate from 90 to its init Position degrees.
    private void GrabberControls() {
        if(gamepad1.right_bumper)
        {
            Grabber.setPosition(GrabberOpen);
        }
        if(gamepad1.left_bumper)
        {
            Grabber.setPosition(GrabberClosed);
        }
    }
    // This method allows the pivot servo to rotate from the initial position of Up to the grabbing position of Down.
    // This method allows to the grabber to collect Specimens in the up position and samples in the down position
    private void Pivot() {
        if(gamepad1.right_trigger >= 0.5)
        {
            Pivot.setPosition(PivotDown);
        }
        if(gamepad1.left_trigger >= 0.5)
        {
            Pivot.setPosition(PivotUp);
        }
    }
    // This method allows the Extension Servo to Extend and retract the Slide rail in a horizontal position
    private void Extension() {
        if(gamepad2.a)
        {
            Extension.setPosition(ExtensionIn);
        } else if (gamepad2.y) {
            Extension.setPosition(ExtensionOut);
        }
        else {
            Extension.setPosition(ExtensionInitPosition);
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
        telemetry.addData("Position", Grabber.getPosition());
        telemetry.addData("Direction", Grabber.getDirection());
        telemetry.addData("Controller", Grabber.getController());

        telemetry.addData("Position", Pivot.getPosition());
        telemetry.addData("Direction", Pivot.getDirection());
        telemetry.addData("Controller", Pivot.getController());

        telemetry.addData("Position", Extension.getPosition());
        telemetry.addData("Direction", Extension.getDirection());
        telemetry.addData("Controller", Extension.getController());
    }

}
