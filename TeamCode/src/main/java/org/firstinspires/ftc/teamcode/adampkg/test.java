package org.firstinspires.ftc.teamcode.adampkg;

//import com.qualcomm.robotcore.eventloop.opmode.OpModeInternal;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class test {
    double liftDirection = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;

    public test() {
    }

    public void Lift( Gamepad gamepad1)
    {
        if(gamepad1.y)
        {
            liftDirection = 1;
        }
        else if(gamepad1.a)
        {
            liftDirection = -1;
        }
        else {
            liftDirection = 0;
        }
        liftRight.setPower(liftDirection);
        liftLeft.setPower(liftDirection);

        //return liftDirection;
    }
}