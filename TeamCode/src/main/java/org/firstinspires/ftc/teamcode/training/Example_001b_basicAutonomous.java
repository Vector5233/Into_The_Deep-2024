package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Robot Config
 * Control Hub
 *
 * Expansion Hub
 */

@Disabled
@Autonomous (group="secondary", name="name")

public class Example_001b_basicAutonomous extends LinearOpMode {
    //global variables
    @Override
    public void runOpMode() throws InterruptedException {
        //define global variables
        waitForStart();

        //sleep(500);
        requestOpModeStop();

    }

    //functions here
}
