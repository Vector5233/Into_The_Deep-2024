package org.firstinspires.ftc.teamcode.christman;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class BASICTELEOPSTRUCTURE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method inits the hardware, motors, servos, and sensors
        // single command to run once
        while(!isStarted()){
            //camera methods before we start
        }
        waitForStart();
        while(opModeIsActive()){
            //methods
        }
    }
    public void initHardware(){}
}
