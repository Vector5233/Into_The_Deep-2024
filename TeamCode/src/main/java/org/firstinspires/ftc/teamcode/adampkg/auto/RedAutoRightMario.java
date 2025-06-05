package org.firstinspires.ftc.teamcode.adampkg.auto;
// ... other imports ...

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.adampkg.teleop.RobotBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

@Autonomous(name="RedAutoRightMario", group="VECTORAUTO")
public class RedAutoRightMario extends LinearOpMode {
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
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8,
        DRIVE_TO_TARGET_9,
        DRIVE_TO_TARGET_10,
        DRIVE_TO_TARGET_11,
        DRIVE_TO_TARGET_12,
        DRIVE_TO_TARGET_13,
        DRIVE_TO_TARGET_14;
    }
    boolean liftsRanUp = false;
    boolean liftsRanDown = false;

    final RobotBase robotBase = new RobotBase();
    // ... Pose2D constants ...
    static final Pose2D REDRIGHT_INIT = new Pose2D(DistanceUnit.MM,-120,1401,AngleUnit.DEGREES,90);
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,-120,656,AngleUnit.DEGREES,90);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -120, 700, AngleUnit.DEGREES, 90);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,-920 , 700, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.MM,-920   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.MM,-1100   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_6 = new Pose2D(DistanceUnit.MM,-1100   , 1200, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_7 = new Pose2D(DistanceUnit.MM,-1100   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_8 = new Pose2D(DistanceUnit.MM,-1402   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_9 = new Pose2D(DistanceUnit.MM,-1402   , 1200, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_10 = new Pose2D(DistanceUnit.MM,-1402   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_11 = new Pose2D(DistanceUnit.MM,-1469   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_12 = new Pose2D(DistanceUnit.MM,-1469   , 1200, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_13 = new Pose2D(DistanceUnit.MM,-1469   , 0, AngleUnit.DEGREES,90);
    static final Pose2D TARGET_14 = new Pose2D(DistanceUnit.MM,-900   , 0, AngleUnit.DEGREES,90);


    // *** APRILTAG LOCALIZATION DECLARATIONS ***
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean aprilTagInitialized = false;

    //Wesley tag id case system
    public Pose2D cameraPosToRealPos(double cameraX, double cameraY, double heading, int id, int tagX, int tagY){
        switch(id){
            case 11:
            case 16:
                return new Pose2D(DistanceUnit.MM, tagX+cameraY, tagY-cameraX, AngleUnit.DEGREES, heading);
            case 13:
            case 14:
                return new Pose2D(DistanceUnit.MM, tagX-cameraY, tagY+cameraX, AngleUnit.DEGREES, heading);
            case 12:
                return new Pose2D(DistanceUnit.MM, tagX-cameraX, tagY-cameraY, AngleUnit.DEGREES, heading);
            case 15:
                return new Pose2D(DistanceUnit.MM, tagX+cameraX, tagY+cameraY, AngleUnit.DEGREES, heading);
            default:
                return null;
        }
    }
    private static class TagInfo {
        int id;
        double x;
        double y;
        double z;
        double yaw;
        public TagInfo(int id, double x, double y, double z, double yaw) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.z = z;
            this.yaw = yaw;
        }
    }
    private TagInfo[] knownTagLocations = {
            new TagInfo(11, -1791, 1189, 0, 90),
            new TagInfo(12, 0, 1791, 0, 0),
            new TagInfo(13, 1791, 1189, 0, -90),
            new TagInfo(14, 1791, -1189, 0, 270),
            new TagInfo(15, 0, -1791, 0, 180),
            new TagInfo(16, -1791, -1189, 0, 90),
    };

    //Use mm
    private double cameraToRobotOffsetX = 159;
    private double cameraToRobotOffsetY = 150;
    private double cameraToRobotOffsetZ = 1;
    private double cameraToRobotOffsetYawDegrees = 90;
    private double cameraToRobotOffsetPitchDegrees = 1;
    private double cameraToRobotOffsetRollDegrees = 1;

    private double estimatedRobotX_AT = 0.0;
    private double estimatedRobotY_AT = 0.0;
    private double estimatedRobotHeading_AT = 0.0;
    private boolean aprilTagDetected = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();




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

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(200, 230); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        sleep(2000);
        odo.setPosition(REDRIGHT_INIT);


        nav.initializeMotors();
        nav.setXYCoefficients(0.03 ,0.000,0.01,DistanceUnit.MM,50);
        nav.setYawCoefficients(1,0,0.1, AngleUnit.DEGREES,360);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odo Position", data);
        waitForStart();
        double fps = visionPortal.getFps();  // Method may vary slightly based on SDK
        telemetry.addData("Camera FPS", fps);
        telemetry.update();
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
        waitLifts(100);
    }
    public void LowerLift()
    {
        runLiftsToPos(liftsLowPos);
        waitLifts(100);
        robotBase.OpenPincher();

        waitLifts(200);
        runLiftsToPos(liftsBottom);
        waitLifts(100);
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
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)

                .build();
        aprilTag.setDecimation(1); // Adjust as needed

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "kyleCamLeft")) // Use your camera name
                .addProcessor(aprilTag)
                .setShowStatsOverlay(true)
                .addProcessor(new MyMonochromeProcessor())
                .setCameraResolution(new Size(320, 180)) // 👈 Example resolution
                .build();

        // Optional: Set manual exposure if using a webcam
        setManualExposure(6, 250);
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

    }

    private void setManualExposure(int exposureMS, int gain) { /* ... (your existing setManualExposure function) ... */ }

    public void stateMachine(StateMachine stateMachine) {
        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Odo Position", data);

            // *** GET APRILTAG DATA AND ESTIMATE POSE ***
            if (aprilTagInitialized) {
                updateAprilTagPoseEstimate(); // Call this function periodically

                if (aprilTagDetected) {
                    // odo.setPosition();
                }
            }

            switch (stateMachine) {

                case WAITING_FOR_START:
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
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
                        waitLifts(100);
                        if (!liftsRanDown) {
                            LowerLift();
                            telemetry.addLine("Called LowerLift()");
                            robotBase.initServos(hardwareMap);
                            liftsRanDown = true;
                        }
                        telemetry.addLine("at position #1!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, 1.0, 0.1, telemetry)) {
                        robotBase.initServos(hardwareMap);
                        telemetry.addLine("at position #2!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if (nav.driveTo(odo.getPosition(), TARGET_3, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #3!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if (nav.driveTo(odo.getPosition(), TARGET_4, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #4!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if (nav.driveTo(odo.getPosition(), TARGET_5, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if (nav.driveTo(odo.getPosition(), TARGET_6, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    if (nav.driveTo(odo.getPosition(), TARGET_7, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if (nav.driveTo(odo.getPosition(), TARGET_8, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_9;
                    }
                    break;
                case DRIVE_TO_TARGET_9:
                    if (nav.driveTo(odo.getPosition(), TARGET_9, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_10;
                    }
                    break;
                case DRIVE_TO_TARGET_10:
                    if (nav.driveTo(odo.getPosition(), TARGET_10, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_11;
                    }
                    break;
                case DRIVE_TO_TARGET_11:
                    if (nav.driveTo(odo.getPosition(), TARGET_11, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_12;
                    }
                    break;
                case DRIVE_TO_TARGET_12:
                    if (nav.driveTo(odo.getPosition(), TARGET_12, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_13;
                    }
                    break;
                case DRIVE_TO_TARGET_13:
                    if (nav.driveTo(odo.getPosition(), TARGET_13, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_14;
                    }
                    break;
                case DRIVE_TO_TARGET_14:
                    if (nav.driveTo(odo.getPosition(), TARGET_14, 1.0, 0.1, telemetry)) {
                        telemetry.addLine("at position #5!");
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
                case AT_TARGET:
                    nav.Stop();
                    break;
            }

            telemetry.addData("current state:",stateMachine);

            if (aprilTagDetected) {
                //telemetry.addData("AT Est. X", String.format("%.1f", estimatedRobotX_AT));
                //telemetry.addData("AT Est. Y", String.format("%.1f", estimatedRobotY_AT));
                //telemetry.addData("AT Est. Heading", String.format("%.1f", estimatedRobotHeading_AT));
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
        telemetry.addData("Total Detections", currentDetections.size());

        double sumRobotX = 0.0;
        double sumRobotY = 0.0;
        double sumRobotHeading = 0.0;
        int validDetections = 0;
        aprilTagDetected = false;

        for (AprilTagDetection detection : currentDetections) {
            TagInfo knownTag = null;
            for (TagInfo info : knownTagLocations) {
                telemetry.addData("Detected Tag", detection.id);
                telemetry.addData("Info", info.id);
                if (info.id == detection.id) {
                    knownTag = info;
                    Pose2D newRotationCoords = getRobotPoseFromTag(detection.ftcPose.x * 25.4 +cameraToRobotOffsetX, detection.ftcPose.y * 25.4 +cameraToRobotOffsetY, detection.ftcPose.yaw, knownTag);
                    odo.setPosition(newRotationCoords);
                    Pose2D pos = odo.getPosition();
                    String realData = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                    String cameraData = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", detection.ftcPose.x*25.4+cameraToRobotOffsetX, detection.ftcPose.y*25.4+cameraToRobotOffsetY, detection.ftcPose.yaw);
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("LocalCamX:", detection.ftcPose.x*25.4+cameraToRobotOffsetX);
                    packet.put("LocalCamY:", detection.ftcPose.y*25.4+cameraToRobotOffsetY);
                    packet.put("LocalCamHeading:", detection.ftcPose.yaw);
                    packet.put("EstWorldX:", newRotationCoords.getX(DistanceUnit.MM));
                    packet.put("EstWorldY:", newRotationCoords.getY(DistanceUnit.MM));
                    packet.put("EstHeading:", newRotationCoords.getHeading(AngleUnit.DEGREES));
                    //packet.put("Odo Coords", realData);
                    packet.put("Camera values:", cameraData);
                    packet.put("OdoX:", pos.getX(DistanceUnit.MM));
                    packet.put("OdoY:", pos.getY(DistanceUnit.MM));
                    packet.put("OdoHeading:", pos.getHeading(AngleUnit.DEGREES));
                    dashboard.sendTelemetryPacket(packet);

                    telemetry.addData("Odo Coords", realData);
                    telemetry.addData("Camera values:", cameraData);
                    break;
                }
            }

            if (knownTag != null) {
                validDetections++;
                telemetry.addLine("Matched known tag!");
                aprilTagDetected = true;
            }
            else {
                telemetry.addLine("Tag ID not in knownTagLocations");
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
    private Pose2D getRobotPoseFromTag(double camX, double camY, double camYaw, TagInfo tag) {
        double yawRad = Math.toRadians(tag.yaw);
        double rotX = camX * Math.cos(yawRad) - camY * Math.sin(yawRad);
        double rotY = camX * Math.sin(yawRad) + camY * Math.cos(yawRad);
        double robotX = tag.x - rotX;
        double robotY = tag.y - rotY;
        double offsetXGlobal = cameraToRobotOffsetX * Math.cos(Math.toRadians(camYaw)) - cameraToRobotOffsetY * Math.sin(Math.toRadians(camYaw));
        double offsetYGlobal = cameraToRobotOffsetX * Math.sin(Math.toRadians(camYaw)) + cameraToRobotOffsetY * Math.cos(Math.toRadians(camYaw));
        double robotHeading = AngleUnit.normalizeDegrees(tag.yaw - camYaw);
        telemetry.addData("Estimated location from tags:", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}",robotX,robotY,robotHeading));

        return new Pose2D(DistanceUnit.MM, robotX+offsetXGlobal, robotY+offsetYGlobal-100, AngleUnit.DEGREES, robotHeading);
    }
}

class MyMonochromeProcessor implements VisionProcessor {
    private Mat gray = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // No special initialization required for grayscale
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2GRAY); // 👈 THIS is what makes it "stick"
        return null; // Replace with data you want to return, if any
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No drawing needed for grayscale, unless debugging or overlays are desired
    }
}
