package org.firstinspires.ftc.teamcode.adampkg.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.adampkg.teleop.RobotBase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;

@Autonomous(name="RedLeftPLUSTAGS", group="VECTORAUTO")

public class RedAutoPLUSTAGS extends LinearOpMode {
//@Disabled
GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    private DcMotor liftLeft;
    private DcMotor liftRight;
    int liftsBottom = 0;
    int liftsLowPos = 2050;
    int liftsHighPos =  2850;
    double liftRightPower = 1.0;
    double liftLeftPower = 1.0;

    public Pose2D cameraPosToRealPos(double cameraX, double cameraY, double heading, int id, int tagX, int tagY){
        double xOutput = 0;
        double yOutput = 0;
        switch(id){
            case 11:
            case 16:

        }
        return new Pose2D(DistanceUnit.MM,xOutput,yOutput,AngleUnit.DEGREES,);
    }


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

    AprilTagDetection detectedTag = null;

    static final Pose2D REDRIGHT_INIT = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.MM,-715,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.MM, -550, -900, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.MM,-1400 , -400, AngleUnit.DEGREES,0);
   // @Override //1300

    aprilTagFieldLocation tag11 = new aprilTagFieldLocation(-1828, 1219, 11);
    aprilTagFieldLocation tag12 = new aprilTagFieldLocation(0, 1828, 12);
    aprilTagFieldLocation tag13 = new aprilTagFieldLocation(1828, 1219, 13);
    aprilTagFieldLocation tag14 = new aprilTagFieldLocation(1828, -1219, 14);
    aprilTagFieldLocation tag15 = new aprilTagFieldLocation(0, -1828, 15);
    aprilTagFieldLocation tag16 = new aprilTagFieldLocation(-1828, -1219, 16);
    aprilTagFieldLocation[] fieldLocations = {tag11, tag12, tag13, tag14, tag15, tag16};

    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(200, 230); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        odo.setPosition(REDRIGHT_INIT);

        nav.initializeMotors();
        nav.setXYCoefficients(0.03 ,0.000,0.0,DistanceUnit.MM,50);
        nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;
        
        AprilTagDetection detectedTag = null;
        startCameraProcessing(detectedTag);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        initMotors();
        robotBase.initServos(hardwareMap);
        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();
        stateMachine(stateMachine);
    }
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

        waitLifts(2000);
        runLiftsToPos(liftsBottom);
        waitLifts(2000);
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
    public void stateMachine(StateMachine stateMachine)
    {
        while (opModeIsActive()) {
            odo.update();
            if(stateMachine == StateMachine.WAITING_FOR_START){
                stateMachine = StateMachine.DRIVE_TO_TARGET_1;


            }
            if (stateMachine == StateMachine.DRIVE_TO_TARGET_1) {

                if (nav.driveTo(odo.getPosition(), TARGET_1, 0.5 , 2, telemetry)) {
                    robotBase.initServos(hardwareMap);
                    if(liftsRanUp == false)
                    {
                        robotBase.initServos(hardwareMap);
                        RaiseLift();
                        liftsRanUp = true;
                    }
                    waitLifts(2000);
                    if(liftsRanDown == false)
                    {
                        LowerLift();
                        robotBase.initServos(hardwareMap);
                        liftsRanDown = true;
                    }
                    telemetry.addLine("at position #1!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                }
            }
            if (stateMachine == StateMachine.DRIVE_TO_TARGET_2){
                if (nav.driveTo(odo.getPosition(), TARGET_2, 0.5, 0.1, telemetry)) {
                    robotBase.initServos(hardwareMap);

                    telemetry.addLine("at position #2!");
                    stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                }
            }
            if (stateMachine == StateMachine.DRIVE_TO_TARGET_3){
                if (nav.driveTo(odo.getPosition(), TARGET_3, 0.5, 0.1, telemetry)){
                    telemetry.addLine("at position #3!");
                    stateMachine = StateMachine.AT_TARGET;
                }

            }
            if(stateMachine == StateMachine.AT_TARGET)
            {
                nav.Stop();
            }
            telemetry.addData("current state:",stateMachine);
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            Pose2D heading = new Pose2D(DistanceUnit.MM,0,0,AngleUnit.RADIANS,nav.calculateTargetHeading(pos,TARGET_2));
            telemetry.addData("target heading: ", heading.getHeading(AngleUnit.RADIANS));
            telemetry.update();

        }
    }
    public void startCameraProcessing(AprilTagDetection detectedTag){
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "kyleCamLeft"))
                .build();
        telemetry.addData("detections:", tagProcessor.getDetections());
        while (!isStopRequested() && opModeIsActive())
        {
            if(tagProcessor.getDetections().size() > 0)
            {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                Pose2D tagPose2d = new Pose2D(DistanceUnit.MM, tag.ftcPose.x, tag.ftcPose.y, AngleUnit.DEGREES, tag.ftcPose.yaw);
                int fieldTagX;
                int fieldTagY;
                for(int i=0; i<fieldLocations.length; i++) {
                    if (fieldLocations[i].id == tag.id) {
                        fieldTagX = fieldLocations[i].x;
                        fieldTagY = fieldLocations[i].y;
                        telemetry.addData("fieldTagX", fieldTagX);
                        telemetry.addData("fieldTagY", fieldTagY);
                    }
                }
                //odo.setPosition(tagPose2d);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("Pitch", tag.ftcPose.pitch);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Roll", tag.ftcPose.roll);
                telemetry.update();
            }
        }
    }
}
