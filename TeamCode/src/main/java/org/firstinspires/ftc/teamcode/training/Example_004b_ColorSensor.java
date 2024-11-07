package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**Configuration file
 * Control Hub
 * I2c Port 03: colorSensor
 */

@Disabled
@TeleOp(group = "Primary", name = "Color Sensor")
public class Example_004b_ColorSensor extends LinearOpMode {
    private ColorSensor colorSensor;
    double redValue;
    double greenValue;
    double blueValue;
    double alphaValue; // light Intensity
    double targetValue = 50;

    private Servo servoOne;
    double servoOneInitPosition =0.5;
    double servoOnePositionOne =0.0;
    double servoOnePositionTwo= 1.0;
    double servoOnePositionThree= 0.75;
    double servoOnePositionFour= 0.25;
    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            getColor();
            colorTelemetry();
        }
        waitForStart();
        while(opModeIsActive()){
            getColor();
            colorTelemetry();
            teleOpControls();
        }
    }
    public void initHardware(){
            initColorSensor();
            initServoOne();
    }
    public void initServoOne(){
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoOne.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(servoOneInitPosition);
    }

    public void initColorSensor(){
        colorSensor=hardwareMap.get(ColorSensor.class, "colorSensor");
    }
    public void getColor(){
        redValue = colorSensor.red();
        greenValue = colorSensor.green();
        blueValue = colorSensor.blue();
        alphaValue = colorSensor.alpha();


    }
    public void colorTelemetry(){
        telemetry.addData("redValue", "%2f", redValue);
        telemetry.addData("greenValue", "%2f", greenValue);
        telemetry.addData("blueValue", "%2f", blueValue);
        telemetry.addData("alphaValue", "%2f", alphaValue);
        telemetry.update();
    }
    public void teleOpControls(){
        if(alphaValue > targetValue){
            servoOne.setPosition(servoOnePositionTwo);
        }
        else if(redValue > targetValue){
            servoOne.setPosition(servoOnePositionOne);
        }
        else if(greenValue > targetValue){
            servoOne.setPosition(servoOnePositionThree);
        }
        else {
            servoOne.setPosition(servoOnePositionFour);

        }
    }

}
