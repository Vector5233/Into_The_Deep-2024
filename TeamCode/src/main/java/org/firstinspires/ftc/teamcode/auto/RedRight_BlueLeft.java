package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name = "RedRight_BlueLeft_Auto 1" , group = "VECTOR_AUTO")

public class RedRight_BlueLeft extends LinearOpMode {
    //@Disabled
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private final DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class
    //Motor Encoders
    private DcMotor liftLeft;
    private DcMotor liftRight;
    int liftsLowPos = 0;
    int liftsHighPos = 50;
    double liftRightPower = 1.0;
    double liftLeftPower = 1.0;
    boolean hasLifted = false;
    // add in servo stuff
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
    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3
    }

    /*  static final Pose2D REDRIGHT_INIT = new Pose2D(DistanceUnit.MM,-220,1340,AngleUnit.DEGREES,-90);

      static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,-220,1340,AngleUnit.DEGREES,0);
      static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -220, 600, AngleUnit.DEGREES, 0);
      static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,-220, 600, AngleUnit.DEGREES,0);
  */
    static final Pose2D REDRIGHT_INIT = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, -705, 0, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -700, 900, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, -1300, 400, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(199, 177); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        odo.setPosition(REDRIGHT_INIT);

        nav.initializeMotors();
        nav.setXYCoefficients(0.40, 0.00, 0.0, DistanceUnit.MM, 12);
        nav.setYawCoefficients(1, 0, 0.0, AngleUnit.DEGREES, 2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
        initMotors();
        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
        stateMachine(stateMachine);
    }

    public void initMotors() {
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setPower(liftLeftPower);
        liftRight.setPower(liftRightPower);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runLiftsToPos(int position) {
        liftLeft.setTargetPosition(position);
        liftRight.setTargetPosition(position);
        telemetry.addData("liftLeft: %2f", liftLeft.getCurrentPosition());
        telemetry.addData("liftRight: %2f", liftRight.getCurrentPosition());

    }

    public void RaiseLift(int position) {
        hasLifted = true;
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runLiftsToPos(position);
        sleep(3000);
        //runLiftsToPos(liftsLowPos);3
    }
    public void LowerLift(int position) {
        hasLifted = true;
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runLiftsToPos(position);
        sleep(3000);
        //runLiftsToPos(liftsLowPos);3
    }
    public void initServos(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        servoPincher = hardwareMap.get(Servo.class, "grabber"); // maps the servo
        servoPincher.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoPincher.setPosition(servoPincherInitPosition); // sets the initial position from the variable above.

        pincherPivot = hardwareMap.get(Servo.class, "pivot"); // maps the servo
        pincherPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        pincherPivot.setPosition(pincherPivotInitPosition); // sets the initial position from the variable above.

        extension = hardwareMap.get(Servo.class, "extension"); // maps the servo
        extension.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        extension.setPosition(extensionInitPosition); // sets the initial position from the variable above.
    }

    public void stateMachine(StateMachine stateMachine) {
        while (opModeIsActive()) {
            odo.update();
            if (stateMachine == StateMachine.WAITING_FOR_START) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;
            }

            if (stateMachine == StateMachine.DRIVE_TO_TARGET_1) {
                if (nav.driveTo(odo.getPosition(), TARGET_1, 0.5, 7)) {
                    if (!hasLifted) {
                        RaiseLift(150);
                        extension.setPosition(extensionRetracted);
                    }
                    sleep(1000);
                    if(liftLeft.getCurrentPosition() == 150 && liftRight.getCurrentPosition() == 150)
                    {
                        servoPincher.setPosition(servoPincherPositionOpen);
                        LowerLift(50);

                    }
                    telemetry.addLine("at position #1!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                }
            }
            if (stateMachine == StateMachine.DRIVE_TO_TARGET_2) {
                if (nav.driveTo(odo.getPosition(), TARGET_2, 0.5, 0.3)) {
                    telemetry.addLine("at position #2!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                }
            }
            if (stateMachine == StateMachine.DRIVE_TO_TARGET_3) {
                if (nav.driveTo(odo.getPosition(), TARGET_3, 0.5, 0.3)) {
                    telemetry.addLine("at position #3!");
                    stateMachine = StateMachine.AT_TARGET;
                }
            }
            if (stateMachine == StateMachine.AT_TARGET) {
                nav.Stop();
            }
            telemetry.addData("liftLeft: %2f", liftLeft.getCurrentPosition());
            telemetry.addData("liftRight: %2f", liftRight.getCurrentPosition());
            telemetry.addData("current state:", stateMachine);
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            Pose2D heading = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, nav.calculateTargetHeading(pos, TARGET_2));
            telemetry.addData("target heading: ", heading.getHeading(AngleUnit.RADIANS));
            telemetry.update();

        }
    }
}
