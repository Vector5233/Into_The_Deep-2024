package org.firstinspires.ftc.teamcode.NEFARIO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Config
@Autonomous(name="Pinpoint Navigation Example", group="Pinpoint")
//@Disabled

public class SensorPinpointDriveToPoint extends LinearOpMode {
// test
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        DRIVE_SIDE_1,
        TURN_1,
        DRIVE_SIDE_2,
        TURN_2,
        DRIVE_SIDE_3,
        TURN_3,
        DRIVE_SIDE_4,
        TURN_4, // Or just return to the start position
        AT_START
    }
    public static double SQUARE_SIDE_LENGTH_MM = 1000.0;
    // Define the target poses for the corners of the square
    static final Pose2D CORNER_1 = new Pose2D(DistanceUnit.MM, SQUARE_SIDE_LENGTH_MM, 0, AngleUnit.DEGREES, 0);
    static final Pose2D CORNER_2 = new Pose2D(DistanceUnit.MM, SQUARE_SIDE_LENGTH_MM, SQUARE_SIDE_LENGTH_MM, AngleUnit.DEGREES, 90);
    static final Pose2D CORNER_3 = new Pose2D(DistanceUnit.MM, 0, SQUARE_SIDE_LENGTH_MM, AngleUnit.DEGREES, 180);
    static final Pose2D CORNER_4 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES,  -90);

    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM, 2000, 20, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, 2600, -20, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM, 2600, -2600, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM, 100, -2600, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM, 100, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // Initialize FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine) {
                case WAITING_FOR_START:
                    // Start the square movement
                    stateMachine = StateMachine.DRIVE_SIDE_1;
                    break;
                case DRIVE_SIDE_1:
                    // Drive to the first corner
                    if (nav.driveTo(odo.getPosition(), CORNER_1, 0.7, 0.5)) { // Adjust speed and hold time
                        telemetry.addLine("Reached Corner 1!");
                        stateMachine = StateMachine.TURN_1;
                    }
                    break;
                case TURN_1:
                    // Turn towards the next corner
                    // Assuming driveTo can handle turns by targeting a new heading
                    if (nav.driveTo(odo.getPosition(), CORNER_2, 0.7, 0.5)) { // Target the next corner to achieve the turn
                        telemetry.addLine("Turned to Corner 2 direction!");
                        stateMachine = StateMachine.DRIVE_SIDE_2;
                    }
                    break;
                case DRIVE_SIDE_2:
                    // Drive to the second corner
                    if (nav.driveTo(odo.getPosition(), CORNER_2, 0.7, 0.5)) {
                        telemetry.addLine("Reached Corner 2!");
                        stateMachine = StateMachine.TURN_2;
                    }
                    break;
                case TURN_2:
                    // Turn towards the next corner
                    if (nav.driveTo(odo.getPosition(), CORNER_3, 0.7, 0.5)) {
                        telemetry.addLine("Turned to Corner 3 direction!");
                        stateMachine = StateMachine.DRIVE_SIDE_3;
                    }
                    break;
                case DRIVE_SIDE_3:
                    // Drive to the third corner
                    if (nav.driveTo(odo.getPosition(), CORNER_3, 0.7, 0.5)) {
                        telemetry.addLine("Reached Corner 3!");
                        stateMachine = StateMachine.TURN_3;
                    }
                    break;
                case TURN_3:
                    // Turn towards the last corner (or start)
                    if (nav.driveTo(odo.getPosition(), CORNER_4, 0.7, 0.5)) {
                        telemetry.addLine("Turned to Corner 4 direction!");
                        stateMachine = StateMachine.DRIVE_SIDE_4;
                    }
                    break;
                case DRIVE_SIDE_4:
                    // Drive back to the start
                    if (nav.driveTo(odo.getPosition(), CORNER_4, 0.7, 0.5)) {
                        telemetry.addLine("Reached Corner 4!");
                        stateMachine = StateMachine.AT_START;
                    }
                    break;
                case AT_START:
                    // The square movement is complete
                    // You can add a final pose check or simply end the OpMode
                    telemetry.addLine("Square movement complete!");
                    requestOpModeStop(); // Stop the OpMode
                    break;
            }


            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();

        }
    }
}