package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotBase {
    double liftDirection = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;
    //ALL THE SERVO STUFF
    Servo servoPincher; // servos go from 0 to 1 rotates 180 degrees
    double servoPincherInitPosition = 0.08; // doubles store a decimal
    double servoPincherPositionClosed = 0.08; //ToDo Test value
    double servoPincherPositionOpen = 0.3;
    Servo pincherPivot; // servos go from 0 to 1 rotates 180 degrees
    double pincherPivotInitPosition = 0.0; // doubles store a decimal
    double pincherPivotDown = 0.7;
    double pincherPivotUp = 0.0;
    Servo extension; // servos go from 0 to 1 rotates 180 degrees
    double extensionInitPosition = 1.0; // doubles store a decimal
    double extensionExtended = 0.0;
    double extensionRetracted = 1.0;

    public RobotBase() {
    }

    public void initServos(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
        servoPincher = hardwareMap.get(Servo.class, "grabber"); // maps the servo
        servoPincher.setDirection(Servo.Direction.FORWARD); // sets the direction of rotation - optional but good practice
        servoPincher.setPosition(servoPincherInitPosition); // sets the initial position from the variable above.

        pincherPivot = hardwareMap.get(Servo.class, "pivot"); // maps the servo
        pincherPivot.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        pincherPivot.setPosition(pincherPivotInitPosition); // sets the initial position from the variable above.

        extension = hardwareMap.get(Servo.class, "extension"); // maps the servo
        extension.setDirection(Servo.Direction.REVERSE); // sets the direction of rotation - optional but good practice
        extension.setPosition(extensionInitPosition); // sets the initial position from the variable above.
    }
    public void OpenPincher()
    {
        servoPincher.setPosition(servoPincherPositionOpen);
    }
    public void ExtendPincher()
    {
        extension.setPosition(extensionExtended);
    }
    public void RetractPincher()
    {

        extension.setPosition(extensionRetracted);
    }
    void Extension(Gamepad gamepad2) {
        if(gamepad2.b)
        {
            extension.setPosition(extensionExtended);
        }
        if(gamepad2.x)
        {
            extension.setPosition(extensionRetracted);
        }
    }
}
