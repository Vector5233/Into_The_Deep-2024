package org.firstinspires.ftc.teamcode.frankfurth;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Config file
 *  Port 00: mortorOne
 *  Port 01: mortorTwo
 *  Port 02: motorThree
 *  Port 00: Servo servoOne
 *  Port 01: Servo servoTwo
 *  Port 02: CRServo servoThree
 *
 */
@Disabled
@TeleOp(group = "Frankfurth", name = "BasicTeleop")
public class SERVOTWO_KF extends LinearOpMode {
    private Servo servoTwo;
    double servoTwoInitPosition =0.50;
    double servoTwoSensitivity =1.0;

    @Override
    public void runOpMode() throws InterruptedException{
        initHardware(); // method inits the hardware motors, servos and sensors
        // single command to run once
        while(!isStarted()){
            // camera methods that
        }
        waitForStart();
        while(opModeIsActive()){
            // method that operate the robot in teleop
        }

    }
    public void initHardware(){

    }
}
