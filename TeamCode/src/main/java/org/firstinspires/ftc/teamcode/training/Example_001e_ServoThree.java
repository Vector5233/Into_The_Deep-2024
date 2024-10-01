package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**Configuration file
 * Control Hub:
 * Servo Port 00: servoOne // must match variable name in CH and in Java
 * Servo Port 01: servoTwo
 * Servo Port 02: crServoThree
 */

//@Disabled
@TeleOp(group = "Primary", name = "Servo with buttons")
public class Example_001e_ServoThree extends LinearOpMode {

    //global variables go below the class name
    private Servo servoOne; // servos go from 0 to 1 rotates 180 degrees
    double servoOneInitPosition =0.5; // doubles store a decimal
    double servoOnePositionOne =0.0;
    double servoOnePositionTwo =1.0;
    int servoOneDelay = 10; //Try not to use sleep/delay in Teleop

    private Servo servoTwo;
    double servoTwoInitPosition =0.50;
    double servoTwoSensitivity =1.0;

    private CRServo crServoThree;
    double crServoThreeZeroPower =0.0;
    double crServoThreePower =1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        while(!isStarted()){
            servoTelemetry();

        }
        waitForStart();
        while(opModeIsActive()){
            servoTelemetry();
            teleOpControls();


        }

    }

    public void initHardware(){
        initServoOne(); // call to the method other methods/ hardware pieces - method stacking
        initServoTwo();
        initServoThree();
    }
    public void servoTelemetry(){
        //telemetry.log().clear(); don't use
        telemetry.addData("Position", servoOne.getPosition());
        telemetry.addData("Position", servoTwo.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.addData("Direction", servoTwo.getDirection());
        telemetry.addData("Servo Three Power", crServoThree.getPower());
        telemetry.addData("Servo three direction", crServoThree.getDirection());
//        telemetry.addData("Controller", servoOne.getController());
//        telemetry.addData("Port Number", servoOne.getConnectionInfo());
//        telemetry.addData("Device Name", servoOne.getDeviceName());
//        telemetry.addData("Manufacture", servoOne.getManufacturer());
//        telemetry.addData("Version", servoOne.getVersion());
//        telemetry.addData("Class", servoOne.getClass());
        telemetry.update();
    }
    public void initServoOne(){
        servoOne = hardwareMap.get(Servo.class, "servoOne"); // maps the servo
        servoOne.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoOne.setPosition(servoOneInitPosition); // sets the initial position from the variable above.
    }
    public  void initServoTwo(){
        servoTwo=hardwareMap.get(Servo.class, "servoTwo");
        servoTwo.setDirection(Servo.Direction.REVERSE);
        servoTwo.setPosition(servoTwoInitPosition);
    }

    public void  initServoThree(){
        crServoThree = hardwareMap.get(CRServo.class, "crServoThree");
        crServoThree.setDirection(CRServo.Direction.FORWARD);
        crServoThree.setPower(crServoThreeZeroPower);
    }

    public void  teleOpControls(){
        if(gamepad1.a){
            servoOne.setPosition(servoOnePositionOne);
        }
        if (gamepad1.b){
            servoOne.setPosition(servoOnePositionTwo);
        }
        if(gamepad1.right_bumper){
            servoOneSlower(servoOnePositionOne, servoOnePositionTwo, servoOneDelay);
        }
        servoTwo.setPosition(servoTwoSensitivity - (gamepad1.right_stick_y * servoTwoSensitivity));

        if(gamepad1.dpad_left){// moves left
            crServoThree.setPower(-crServoThreePower);
        }
        if(gamepad1.dpad_down){// stop
            crServoThree.setPower(crServoThreeZeroPower);
        }
        if(gamepad1.dpad_right){ // moves right
            crServoThree.setPower(crServoThreePower);
        }

    }
    public void servoOneSlower(double startPosition, double endPosition, int delay){
        // maybe for Auto mode.
        double range =((endPosition - startPosition) * 100);
        //for(local variable; conditional; update variable)
        for(int i =0; i<= range; i++){
            servoOne.setPosition(startPosition);
            sleep(delay);
            startPosition = startPosition + 0.01;
        }
    }
}
