package org.firstinspires.ftc.teamcode.NEFARIO;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    // --- Add these if your DriveToPoint class can provide them ---
    // You'll need to modify DriveToPoint to expose these, or calculate them here if possible
    public static double xError = 0; // Placeholder
    public static double yError = 0; // Placeholder
    public static double headingError = 0; // Placeholder
    public static double xPIDOutput = 0; // Placeholder
    public static double yPIDOutput = 0; // Placeholder
    public static double yawPIDOutput = 0; // Placeholder
    // --- End placeholders ---
    // ... (CORNER definitions, etc.) ...
    // Define your PID coefficients here if you want them to be @Config editable
    // Or get them from your 'nav' object if it stores them publicly
    public static double Kp_XY = 0.82; // Example, link to nav.setXYCoefficients
    public static double Ki_XY = 0.0;
    public static double Kd_XY = 0.0;

    public static double Kp_Yaw = 0.0; // Example, link to nav.setYawCoefficients
    public static double Ki_Yaw = 0.0;
    public static double Kd_Yaw = 0.0;
    enum StateMachine {
        WAITING_FOR_START,
        START_POSE,
        DRIVE_SIDE_1,
        TURN_1,
        DRIVE_SIDE_2,
        TURN_2,
        DRIVE_SIDE_3,
        TURN_3,
        DRIVE_SIDE_4,
        TURN_4, // Or just return to the start position
        RETURN_TO_START_ORIENTATION,
        AT_START
    }
    public static double SQUARE_SIDE_LENGTH_MM = 2600.0;
    // Define the target poses for the corners of the square
    static final Pose2D START_POSE = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0); // Define for clarity
    static final Pose2D CORNER_1 = new Pose2D(DistanceUnit.MM, SQUARE_SIDE_LENGTH_MM, 0, AngleUnit.DEGREES, 0);
    static final Pose2D CORNER_2 = new Pose2D(DistanceUnit.MM, SQUARE_SIDE_LENGTH_MM, SQUARE_SIDE_LENGTH_MM, AngleUnit.DEGREES, 90);
    static final Pose2D CORNER_3 = new Pose2D(DistanceUnit.MM, SQUARE_SIDE_LENGTH_MM, SQUARE_SIDE_LENGTH_MM, AngleUnit.DEGREES, 178);
    static final Pose2D CORNER_4 = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES,  -268);
    static final Pose2D RETURN_TO_START = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);

//    static final Pose2D DRIVE_SIDE_1 = new Pose2D(DistanceUnit.MM, 2000, 20, AngleUnit.DEGREES, 0);
//    static final Pose2D DRIVE_SIDE_2 = new Pose2D(DistanceUnit.MM,  2500, 20, AngleUnit.DEGREES, -90);
//    static final Pose2D DRIVE_SIDE_3 = new Pose2D(DistanceUnit.MM, 1000, -1000, AngleUnit.DEGREES, -90);
//    static final Pose2D Drive_Side_4 = new Pose2D(DistanceUnit.MM, 100, -2600, AngleUnit.DEGREES, 90);
//    static final Pose2D Drive_Side_5 = new Pose2D(DistanceUnit.MM, 100, 0, AngleUnit.DEGREES, 0);


    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();

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
        odo.setOffsets(-180.0,120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();

        //nav.setXYCoefficients(Kp_XY,Ki_XY,0.00,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        //telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        //telemetry.addData("Device Scalar", odo.getYawScalar());

        odo.resetPosAndIMU();
        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();
            Pose2D currentPose = odo.getPosition();
            Pose2D targetPose = null; // Will be set in the state machine

           // TelemetryPacket packet = new TelemetryPacket();
            switch (stateMachine) {
                case WAITING_FOR_START:
                    //Start the square movement
                    stateMachine = StateMachine.DRIVE_SIDE_1;
                    break;
                case DRIVE_SIDE_1:
                   // stateMachine = StateMachine.DRIVE_SIDE_1;
                    targetPose = CORNER_1;
                    // Drive to the first corner
                    if (nav.driveTo(currentPose, targetPose, 0.7, 0.5)) { // Adjust speed and hold time
                        telemetry.addLine("Reached Corner 1!");
                        stateMachine = StateMachine.DRIVE_SIDE_2;
                    }
                    break;

//                case TURN_1:
//                    // Turn towards the next corner
//                    // Assuming driveTo can handle turns by targeting a new heading
//                    if (nav.driveTo(odo.getPosition(), CORNER_2, 0.7, 0.5)) { // Target the next corner to achieve the turn
//                        telemetry.addLine("Turned to Corner 2 direction!");
//                        stateMachine = StateMachine.DRIVE_SIDE_2;
//                    }
//                    break;

                case DRIVE_SIDE_2:
                    // Drive to the second corner
                    targetPose = CORNER_2;
                    if (nav.driveTo(odo.getPosition(), CORNER_2, 0.7, 0.5)) {
                        telemetry.addLine("Reached Corner 2!");
                        stateMachine = StateMachine.DRIVE_SIDE_3;
                    }
                    break;

//                case TURN_2:
//                    // Turn towards the next corner
//                    if (nav.driveTo(odo.getPosition(), CORNER_3, 0.7, 0.5)) {
//                        telemetry.addLine("Turned to Corner 3 direction!");
//                        stateMachine = StateMachine.DRIVE_SIDE_3;
//                    }
//                    break;
                    
               case DRIVE_SIDE_3:
                    // Drive to the third corner
                   targetPose = CORNER_3;
                    if (nav.driveTo(odo.getPosition(), CORNER_3, 0.7, 0.5)) {
                        telemetry.addLine("Reached Corner 3!");
                        stateMachine = StateMachine.DRIVE_SIDE_4;
                    }
                    break;

//                case TURN_3:
//                    // Turn towards the last corner (or start)
//                    if (nav.driveTo(odo.getPosition(), CORNER_4, 0.7, 0.5)) {
//                        telemetry.addLine("Turned to Corner 4 direction!");
//                        stateMachine = StateMachine.DRIVE_SIDE_4;
//                    }
//                    break;

                case DRIVE_SIDE_4:
                    // Drive back to the start
                    targetPose = CORNER_4;;
                    if (nav.driveTo(odo.getPosition(), CORNER_4, 0.7, 0.5)) {
                        telemetry.addLine("Reached Corner 4!");
                        stateMachine = StateMachine.RETURN_TO_START_ORIENTATION;                    }

                    break;
                case RETURN_TO_START_ORIENTATION:
                    // Target the true start pose (0,0,0) to correct final orientation
                    targetPose = RETURN_TO_START; // Or START_POSE
                    if (nav.driveTo(currentPose, targetPose, 0.7, 0.5)) { // Speed for turn might be lower
                        telemetry.addLine("Returned to start orientation!");
                        stateMachine = StateMachine.AT_START;
                    }
                    break;
//                    case TURN_4:
//                    // Turn towards the Start (or start)
//                    if (nav.driveTo(odo.getPosition(), CORNER_1, 0.7, 0.5)) {
//                        telemetry.addLine("Turned to Corner 1 direction!");
//                        stateMachine = StateMachine.AT_START;
//                    }
//                    break;
                case AT_START:
                    // The square movement is complete
                    // You can add a final pose check or simply end the OpMode
                    telemetry.addLine("Square movement complete!");
                    leftFrontDrive.setPower(0);
                    rightFrontDrive.setPower(0);
                    leftBackDrive.setPower(0);
                    rightBackDrive.setPower(0);
                    requestOpModeStop();// Stop the OpMode
                    break;
            }
//            break;   case AT_START:
//                requestOpModeStop();
//                break;
//        }



        //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            leftFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            rightFrontDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            leftBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            rightBackDrive.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            double robotXInches = pos.getX(DistanceUnit.MM) / 25.4;
            double robotYInches = pos.getY(DistanceUnit.MM) / 25.4;
            double robotHeadingRad = pos.getHeading(AngleUnit.DEGREES);

            double ROBOT_RADIUS_INCHES = 9; // Example radius

            // --- Populate TelemetryPacket for PID Tuning ---
            packet.put("Current State", stateMachine.toString());

            // Robot Pose
            packet.put("X (mm)", currentPose.getX(DistanceUnit.MM));
            packet.put("Y (mm)", currentPose.getY(DistanceUnit.MM));
            packet.put("Heading (deg)", currentPose.getHeading(AngleUnit.DEGREES));

            if (targetPose != null) {
                packet.put("Target X (mm)", targetPose.getX(DistanceUnit.MM));
                packet.put("Target Y (mm)", targetPose.getY(DistanceUnit.MM));
                packet.put("Target Heading (deg)", targetPose.getHeading(AngleUnit.DEGREES));
            }
            packet.put("LF Power", nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            packet.put("RF Power", nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            packet.put("LB Power", nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            packet.put("RB Power", nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            // Active PID Coefficients (if you want to see them on dashboard)
            // You might need methods in DriveToPoint to retrieve current P, I, D
            // or just display the static Kp_XY, etc. if using @Config
            packet.put("Kp_XY", Kp_XY); // Assuming Kp_XY is the one you used
            packet.put("Ki_XY", Ki_XY);
            packet.put("Kd_XY", Kd_XY);

            // packet.put("Kp_Yaw", Kp_Yaw);
            // packet.put("Ki_Yaw", Ki_Yaw);
            // packet.put("Kd_Yaw", Kd_Yaw);

            // You can also draw a line indicating the robot's heading
             robotXInches = currentPose.getX(DistanceUnit.MM) / 25.4;
             robotYInches = currentPose.getY(DistanceUnit.MM) / 25.4;
             robotHeadingRad = currentPose.getHeading(AngleUnit.RADIANS);
             ROBOT_RADIUS_INCHES = 9;
            Pose2D actualCurrentPose = odo.getPosition(); // Get current pose again if needed
            double actualRobotXInches = actualCurrentPose.getX(DistanceUnit.MM) / 25.4;
            double actualRobotYInches = actualCurrentPose.getY(DistanceUnit.MM) / 25.4;
            double actualRobotHeadingRad = actualCurrentPose.getHeading(AngleUnit.RADIANS);

            packet.fieldOverlay()
                    .setFill("blue") // Actual robot in blue
                    .fillCircle(actualRobotXInches, actualRobotYInches, ROBOT_RADIUS_INCHES)
                    .setStroke("red")
                    .strokeLine(actualRobotXInches, actualRobotYInches,
                            actualRobotXInches + 12 * Math.cos(actualRobotHeadingRad), // 12 is heading line length
                            actualRobotYInches + 12 * Math.sin(actualRobotHeadingRad));
            // --- End Optional: Actual robot position ---


            // Standard Telemetry (optional, if you want it on DS too)
            telemetry.addData("X", currentPose.getX(DistanceUnit.MM));
            telemetry.addData("Y", currentPose.getY(DistanceUnit.MM));
            telemetry.addData("H", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Robot X inches", robotXInches);
            telemetry.addData("Robot Y inches", robotYInches);
            telemetry.addData("Robot Heading Rad", robotHeadingRad);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();

        }
    }
}
