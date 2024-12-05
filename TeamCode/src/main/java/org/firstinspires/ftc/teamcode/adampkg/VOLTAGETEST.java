package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**Config File
 * Port 00: motorOne
 * Port 01: motorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 */
@Disabled
@TeleOp(group = "Qureshi", name = "AQVoltageTest")
public class VOLTAGETEST extends LinearOpMode {
    private VoltageSensor batteryVoltageSensor;


    @Override
    public void runOpMode() throws InterruptedException {
    initHardware();
    while(!isStarted())
    {
        //This is a comment
    }
    waitForStart();
    while(opModeIsActive())
    {
        //methods
        sensorTelemetry();
    }

    }
    public void initHardware()
    {
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
