package org.firstinspires.ftc.teamcode.adampkg.auto;
// ... other imports ...

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.adampkg.teleop.RobotBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

@Autonomous(name="RedAutoPLUSTAGS", group="VECTORAUTO")
public class RedAutoLeftTags extends LinearOpMode {
    //@Disabled
    GoBildaPinpointDriver odo;
    private DriveToPoint nav = new DriveToPoint(this);
    private DcMotor liftLeft;
    private DcMotor liftRight;
    int liftsBottom = 0;
    int liftsLowPos = 1900;
    int liftsHighPos =  2850;

    double liftRightPower = 1.0;
    double liftLeftPower = 1.0;
    enum StateMachine{
        WAITING_FOR_START,
        AT_TARGET,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3;
    }
    boolean liftsRanUp = false;
    boolean liftsRanDown = false;

    final RobotBase robotBase = new RobotBase();
    // ... Pose2D constants ...
    static final Pose2D REDRIGHT_INIT = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,-745,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -550, -950, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,-1400 , -400, AngleUnit.DEGREES,0);

    // *** APRILTAG LOCALIZATION DECLARATIONS ***
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean aprilTagInitialized = false;

    private static class TagInfo {
        int id;
        double x;
        double y;
        double z;
        double yaw;
        public TagInfo(int id, double x, double y, double z, double yaw) { /* ... */ }
    }
    private TagInfo[] knownTagLocations = { /* ... */ };

    //Use mm
    private double cameraToRobotOffsetX = 1;
    private double cameraToRobotOffsetY = 1;
    private double cameraToRobotOffsetZ = 1;
    private double cameraToRobotOffsetYawDegrees = 1;
    private double cameraToRobotOffsetPitchDegrees = 1;
    private double cameraToRobotOffsetRollDegrees = 1;

    private double estimatedRobotX_AT = 0.0;
    private double estimatedRobotY_AT = 0.0;
    private double estimatedRobotHeading_AT = 0.0;
    private boolean aprilTagDetected = false;


    @Override
    public void runOpMode() {
        // ... odometry and navigation initialization ...
        initMotors();
        robotBase.initServos(hardwareMap);

        // *** INITIALIZE APRILTAGS ***
        initAprilTag();
        aprilTagInitialized = true;

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        // ... other telemetry ...
        telemetry.update();

        waitForStart();
        resetRuntime();
        stateMachine(stateMachine);
    }

    // ... initMotors(), runLiftsToPos(), RaiseLift(), LowerLift(), waitLifts(), motorTelemetry() ...
    public void initMotors()
    {
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setPower(liftLeftPower);
        liftRight.setPower(liftRightPower);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);

        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void runLiftsToPos(int position)
    {
        robotBase.initServos(hardwareMap);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setTargetPosition(position);
        liftRight.setTargetPosition(position);
    }
    public void RaiseLift()
    {
        runLiftsToPos(liftsHighPos);
        waitLifts(2000);
    }
    public void LowerLift()
    {
        runLiftsToPos(liftsLowPos);
        waitLifts(2000);
        robotBase.OpenPincher();

        waitLifts(500);
        runLiftsToPos(liftsBottom);
        waitLifts(1000);
    }
    public void waitLifts(int holdTime)
    {
        while(opModeIsActive() && liftRight.isBusy() && liftLeft.isBusy())
        {
            motorTelemetry();
        }
        sleep(holdTime);
    }
    public void  motorTelemetry(){
        telemetry.addData("leftLift","Encoder: %2d, Power: %2f", liftLeft.getCurrentPosition(), liftLeft.getPower());
        telemetry.addData("leftRight","Encoder: %2d, Power: %2f", liftRight.getCurrentPosition(), liftRight.getPower());
        telemetry.update();
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2); // Adjust as needed

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "kyleCamLeft")) // Use your camera name
                .addProcessor(aprilTag)
                .build();

        // Optional: Set manual exposure if using a webcam
        setManualExposure(6, 250);
    }

    private void setManualExposure(int exposureMS, int gain) { /* ... (your existing setManualExposure function) ... */ }

    public void stateMachine(StateMachine stateMachine) {
        while (opModeIsActive()) {
            odo.update();

            // *** GET APRILTAG DATA AND ESTIMATE POSE ***
            if (aprilTagInitialized) {
                updateAprilTagPoseEstimate(); // Call this function periodically
            }

            switch (stateMachine) {
                case WAITING_FOR_START:
                    stateMachine = RedAutoLeftTags.StateMachine.DRIVE_TO_TARGET_1;
                    if (!liftsRanUp) {
                        robotBase.initServos(hardwareMap);
                        RaiseLift();
                        liftsRanUp = true;
                    }
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), TARGET_1, 0.3, 2, telemetry)) {
                        robotBase.initServos(hardwareMap);
                        telemetry.addLine("In drive to target 1");
                        waitLifts(1000);
                        if (!liftsRanDown) {
                            LowerLift();
                            telemetry.addLine("Called LowerLift()");
                            robotBase.initServos(hardwareMap);
                            liftsRanDown = true;
                        }
                        telemetry.addLine("at position #1!");
                        stateMachine = RedAutoLeftTags.StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 0.8, 0.1, telemetry)) {
                        robotBase.initServos(hardwareMap);
                        telemetry.addLine("at position #2!");
                        stateMachine = RedAutoLeftTags.StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TARGET_3, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #3!");
                        stateMachine = RedAutoLeftTags.StateMachine.AT_TARGET;
                    }
                    break;
                case AT_TARGET:
                    nav.Stop();
                    break;
            }

            telemetry.addData("current state:",stateMachine);
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Odo Position", data);
            if (aprilTagDetected) {
                telemetry.addData("AT Est. X", String.format("%.1f", estimatedRobotX_AT));
                telemetry.addData("AT Est. Y", String.format("%.1f", estimatedRobotY_AT));
                telemetry.addData("AT Est. Heading", String.format("%.1f", estimatedRobotHeading_AT));
            } else {
                telemetry.addLine("No AprilTag Detected for Localization");
            }
            // ... other telemetry ...
            telemetry.update();
        }
    }

    /**
     * Updates the robot's estimated pose based on detected AprilTags.
     */
    private void updateAprilTagPoseEstimate() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double sumRobotX = 0.0;
        double sumRobotY = 0.0;
        double sumRobotHeading = 0.0;
        int validDetections = 0;
        aprilTagDetected = false;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                TagInfo knownTag = null;
                for (TagInfo info : knownTagLocations) {
                    if (info.id == detection.id) {
                        knownTag = info;
                        break;
                    }
                }

                if (knownTag != null) {
                    // Get tag pose relative to camera
                    double tagRelativeX = detection.ftcPose.x;    // Forward from camera
                    double tagRelativeY = detection.ftcPose.y;    // Left from camera
                    double tagRelativeZ = detection.ftcPose.z;    // Up from camera
                    double tagRelativeYaw = detection.ftcPose.yaw;

                    // Apply camera offset to estimated robot position
                    // Adjust tag's position from camera space to robot space using camera offset
                    double tagXRobot = tagRelativeX + cameraToRobotOffsetX;
                    double tagYRobot = tagRelativeY + cameraToRobotOffsetY;
                    double tagZRobot = tagRelativeZ + cameraToRobotOffsetZ;

                    // Adjust yaw (rotation about Z) for camera mounting angle
                    double adjustedYaw = tagRelativeYaw + cameraToRobotOffsetYawDegrees;

                    // Estimate robot's position in the world
                    double estRobotWorldX = knownTag.x - tagYRobot;
                    double estRobotWorldY = knownTag.y + tagXRobot;
                    double estRobotWorldHeading = AngleUnit.normalizeDegrees(knownTag.yaw - adjustedYaw);

                    sumRobotX += estRobotWorldX;
                    sumRobotY += estRobotWorldY;
                    sumRobotHeading += estRobotWorldHeading;
                    validDetections++;
                    aprilTagDetected = true;
                }
            }
        }

        if (validDetections > 0) {
            estimatedRobotX_AT = sumRobotX / validDetections;
            estimatedRobotY_AT = sumRobotY / validDetections;
            estimatedRobotHeading_AT = sumRobotHeading / validDetections;
        } else {
            aprilTagDetected = false;
        }
    }

    public static Quaternion eulerToQuaternion(double roll, double pitch, double yaw) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);

        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;

        return new Quaternion((float) x, (float) y, (float) z, (float) w, System.nanoTime());
    }

}
