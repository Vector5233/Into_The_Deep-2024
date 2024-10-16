package org.firstinspires.ftc.teamcode.training;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Configuration file
 * Control Hub
 * I2C Port 00: IMU // this is the build in one.
 */

//@Disabled
@TeleOp(group = "Primary", name = "Imu-Gyro Use")
public class Example_004f_Gyro extends LinearOpMode {

    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            imuTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            imuTelemetry();
        }
    }

    public void initHardware() {
        initImu();
    }
    public void  initImu(){


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


    }
    public double getAngle(){
        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return heading;
    }
    public void imuTelemetry(){
        telemetry.addData("Robot Angle", getAngle());  // Top to bottom
        telemetry.update();

    }
}
