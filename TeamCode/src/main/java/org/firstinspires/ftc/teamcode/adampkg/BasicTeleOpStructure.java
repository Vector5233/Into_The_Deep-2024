package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**Config File
 * Port 00: motorOne
 * Port 01: motorTwo
 * Port 02: motorThree
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 */
@Disabled
@TeleOp(group = "Qureshi", name = "BasicTeleopStarter")
public class BasicTeleOpStructure extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
    initHardware();
    while(!isStarted())
    {
        //camera methods that
        //This is a comment
    }
    waitForStart();
    while(opModeIsActive())
    {
        //methods
    }

    }
    public void initHardware()
    {

    }

}
