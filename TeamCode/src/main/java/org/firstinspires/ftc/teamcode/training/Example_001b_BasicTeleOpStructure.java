package org.firstinspires.ftc.teamcode.training;
// Cover packages (directories), creating a java class, and refactoring
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


// InLine Comments
/*
Block Comments can collapse...
 */


/** Java Doc
 * Configuration file
 *
 */

@Disabled // does not show up on DriverStation- must be commented out before uploading
@TeleOp(group = "Primary", name = "ShorName")
public class Example_001b_BasicTeleOpStructure extends LinearOpMode {
    //public - An access modifier indicating this method can be called from outside this class
    //class - A collection/blueprint of members/fields/global variables and methods and functions
    //Classes are named in UpperCamelCase
    //extends (inheritance) - This class 'inherits' or 'is the child of' LinearOpMode
    //LinearOpMode - linear logic
    //Ctrl-Click on LinearOpMode to view the class
    @Override

    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){}
        waitForStart();
        while(opModeIsActive()){}
    }
    public void initHardware(){}
}
