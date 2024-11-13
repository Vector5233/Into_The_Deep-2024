
package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Config File
 * Port 00: motorOne MFL
 * Port 01: motorTwo MFR
 * Port 02: motorThree MBL
 * Port 03: motorFour MBR
 *
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 *
 */
//@Disabled
@TeleOp(group = "Qureshi", name = "BasicTeleop")
public class MechAnimAQ extends LinearOpMode {
    double liftDirection = 0;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor liftRight;
    DcMotor liftLeft;

    //a game pad = a controller

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            teleOpControls();
        }

    }
    public void Lift()
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
    public void initHardware() {
        initDrive();
    }

    private void initDrive() {
        frontLeft = hardwareMap.get(DcMotor.class,"leftFront");
        frontRight = hardwareMap.get(DcMotor.class,"rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class,"rightBack");
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        // backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }
    public void driveTrain()
    {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        frontLeft.setPower((ly + lx + rx) / max);
        frontRight.setPower((ly - lx - rx) / max);
        backLeft.setPower((ly - lx + rx) / max);
        backRight.setPower((ly + lx - rx) / max);
    }

    public void teleOpControls() {
        driveTrain();
        Lift();
        liftDirection = 0;
    }
}
