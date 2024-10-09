package org.firstinspires.ftc.teamcode.christman;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Port 00: motorOne
 * Port 01: motorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 *
 */
//@Disabled
public class VOLTAGETEST extends LinearOpMode {

    private VoltageSensor batteryVoltageSensor;
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method inits the hardware, motors, servos, and sensors
        // single command to run once
        while(!isStarted()){
            //camera methods before we start
        }
        waitForStart();
        while(opModeIsActive()){
            //methods for teleop
            sensorTelemetry();
        }
    }
    public void initHardware(){
        initVoltageSensor();
    }
    public void initVoltageSensor(){
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    public void sensorTelemetry(){
        telemetry.addData("get voltage", batteryVoltageSensor.getVoltage());
        telemetry.update();
    }
}
