package org.firstinspires.ftc.teamcode.zalkin;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Configuration file
 * Port 00: motorOne
 * Port 01: motorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo contServoOne
 *
 * @noinspection SpellCheckingInspection
 */
//@Disabled
@TeleOp(group = "Zalkin", name = "BasicTeleop")
public class BASICTELEOPSTRUCTURE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware(); // method initiates hardware components
        //runs once
        while (!isStarted()) {
            // camera methods that
        }
        waitForStart();
        while (opModeIsActive()) {
            //method that operates teh robot in teleop
        }
    }

    public void initHardware() {
    }
}