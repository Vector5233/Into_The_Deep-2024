package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**Configuration file
 * Control Hub
 * I2C Port 01:distanceSensor
 */

//@Disabled
@TeleOp(group = "Primary", name = "Distance Sensor")
public class Example_004a_DistanceSen extends LinearOpMode {
    private DistanceSensor distanceSensor; // voltage issues - as battery drops you can get error sound with connection
    int distanceSensorTarget = 10;

    private Servo servoOne;
    double servoOneInitPosition =0.5;
    double servoOnePositionOne =0.0;
    double servoOnePositionTwo= 1.0;

    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            distanceTelementry();
        }
        waitForStart();
        while(opModeIsActive()){
            distanceTelementry();
            teleOpControls();
        }
    }
    public void initHardware(){
        initDistanceSensor();
        initServoOne();
    }
    public void initServoOne(){
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoOne.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(servoOneInitPosition);
    }
    public void initDistanceSensor(){
    distanceSensor=hardwareMap.get(DistanceSensor.class, "distanceSensor");

    }
    public void distanceTelementry(){
        telemetry.addData("Distance in CM: ", "%.2f", distanceSensor.getDistance(DistanceUnit.CM)); //gets data
        telemetry.update(); // sends data to the screen.
        //always test with a ruler or tape measure.
    }
    public void teleOpControls(){
        if(distanceSensor.getDistance(DistanceUnit.CM)< distanceSensorTarget){
            servoOne.setPosition(servoOnePositionTwo);
        }else{
            servoOne.setPosition(servoOnePositionOne);
        }
    }


}// class bracket
