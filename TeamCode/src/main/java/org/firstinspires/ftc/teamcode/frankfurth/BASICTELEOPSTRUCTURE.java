package org.firstinspires.ftc.teamcode.frankfurth;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Config file
 *  Port 00: mortorOne
 *  Port 01: mortorTwo
 *  Port 02: motorThree
 *  Port 00: Servo servoOne
 *  Port 01: Servo servoTwo
 *  Port 02: CRServo servoThree
 *
 */
//@Disabled
@TeleOp(group = "Frankfurth", name = "BasicTeleop")
public class BASICTELEOPSTRUCTURE extends LinearOpMode {
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
