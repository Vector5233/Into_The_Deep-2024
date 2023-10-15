package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous - Blue ball", group="Linear Opmode")
public class AutoBlueNear extends AutonomousOpenCV {
    @Override
    public void runOpMode()
    {
        detectionRed = false;
        super.runOpMode();
        // write code for Blue Near
        df.turnToHeading(TURN_SPEED, 90);
df.driveStraight(DRIVE_SPEED, 40, 90);
    }

}