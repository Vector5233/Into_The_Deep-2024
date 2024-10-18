package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**Configuration file
 *
 */

@Disabled
@TeleOp(group = "Primary", name = "Touch Sensor")
public class Example_004c_ToSensor extends LinearOpMode {
    private TouchSensor touchSensor;
    boolean touchSensorIsPressed = true;
    double touchSensorValue;

    private Servo servoOne;
    double servoOneInitPosition =0.5;
    double servoOnePositionOne =0.0;
    double servoOnePositionTwo= 1.0;

    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){
            switchTelemetry();

        }
        waitForStart();
        while(opModeIsActive()){
            switchTelemetry();
            teleOpControls();
        }
    }
    public void initHardware(){
        initTouchSensor();
        initServoOne();
    }
    public void initServoOne(){
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoOne.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(servoOneInitPosition);
    }

    public void initTouchSensor(){
        touchSensor=hardwareMap.get(TouchSensor.class, "touchSensor");
    }
    public void getTouchSensor(){
        touchSensorIsPressed = touchSensor.isPressed();
        touchSensorValue = touchSensor.getValue();
    }
    public  void switchTelemetry(){
        telemetry.addData("Touch Sensor pressed", touchSensorIsPressed);
        telemetry.addData("Touch Sensor Value", "%2f", touchSensorValue);
        telemetry.addData("servoOnePosition","%2f", servoOne.getPosition());
        telemetry.update();

    }
    public void teleOpControls(){
        if(touchSensorIsPressed){
            servoOne.setPosition(servoOnePositionTwo);
        }else {
            servoOne.setPosition(servoOnePositionOne);
        }

    }
}
