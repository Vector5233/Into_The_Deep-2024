package org.firstinspires.ftc.teamcode.training;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Configuration file
 * Control Hub
 * I2C Port 00: IMU // this is the build in one.
 */

//@Disabled
@TeleOp(group = "Primary", name = "Imu-Gyro Use")
public class Example_004f_Gyro extends LinearOpMode {

    private BNO055IMU imu;
    double imuX = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    double imuY = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    double imuZ = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
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
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); // new instance
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES; // settings
        parameters.calibrationDataFile = "SensorBHI260Calibration.json"; // settings
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public void imuTelemetry(){
        telemetry.addData("Pitch-X", "%.2f", imuX);//USB ports to servo ports
        telemetry.addData("Roll-Y", "%.2f", imuY); // Motor ports to sensor ports
        telemetry.addData("Yaw-Z", "%.2f", imuZ);   // Top to bottom
        telemetry.update();

    }
}
