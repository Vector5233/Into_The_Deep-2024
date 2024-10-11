package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**Config File
 * Port 00: motorOne
 * Port 01: motorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 */
//@Disabled
@TeleOp(group = "Qureshi", name = "BasicTeleop")
public class AQSERVOONE extends LinearOpMode {
    private Servo servoOne;
    double servoOneInitPosition = 0.5;
    double servoOnePositionOne = 0.0;
    double servoOnePositionTwo = 1.0;
    int servoOneDelay = 10;
    @Override
    public void runOpMode() throws InterruptedException {
    initHardware();
    while(!isStarted())
    {
servoTelemetry();
    }
    waitForStart();
    while(opModeIsActive())
    {
        //methods
    }

    }
    public void initHardware()
    {
        initServoOne();
    }
    public void servoTelemetry()
    {
        telemetry.addData("Position", servoOne.getPosition());
        telemetry.addData("Direction", servoOne.getDirection());
        telemetry.addData("Controller", servoOne.getController());
        telemetry.addData("Port Number", servoOne.getConnectionInfo());
        telemetry.addData("Device Name", servoOne.getDeviceName());
        telemetry.addData("Version", servoOne.getVersion());
        telemetry.addData("Class", servoOne.getClass());
    }
    public void initServoOne()
    {
        servoOne = hardwareMap.get(Servo.class, "servoOne");
        servoOne.setDirection(Servo.Direction.FORWARD);
        servoOne.setPosition(servoOneInitPosition);
    }
    public void teleOpControls()
    {
        if(gamepad1.a)
        {
            servoOne.setPosition(servoOnePositionOne);
        }
        if(gamepad1.b)
        {
        servoOne.setPosition(servoOnePositionOne);
        }
        if(gamepad1.right_bumper)
        {
            servoOneSlower(servoOnePositionOne, servoOnePositionTwo, servoOneDelay);
        }
    }
    public void servoOneSlower(double startPosition, double endPosition, int delay)
    {
        double range = ((endPosition - startPosition) * 100);
        for(int i = 0; i <= range; i++)
        {
            servoOne.setPosition(startPosition);
            sleep(delay);
            startPosition = startPosition + 0.01;
        }
    }
}
