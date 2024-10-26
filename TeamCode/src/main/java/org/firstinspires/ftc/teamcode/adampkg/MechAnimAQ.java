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
 * Port 00: Servo servoOne
 * Port 01: Servo servoTwo
 * Port 02: CRServo servoThree
 */
//@Disabled
@TeleOp(group = "Qureshi", name = "BasicTeleop")
public class MechAnimAQ extends LinearOpMode {

    DcMotor dcMotorMFL;
    DcMotor dcMotorMFR;
    DcMotor dcMotorMBL;
    DcMotor dcMotorMBR;

    //a game pad = a controller
    double lx = gamepad1.left_stick_x;
    double ly = gamepad1.left_stick_y;
    double rx = gamepad1.right_stick_x;
    double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

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

    public void initHardware() {
        dcMotorMFL = hardwareMap.dcMotor.get("MFL");
        dcMotorMFR = hardwareMap.dcMotor.get("MFR");
        dcMotorMBL = hardwareMap.dcMotor.get("MBL");
        dcMotorMBR = hardwareMap.dcMotor.get("MBR");

        dcMotorMBL.setDirection(DcMotorSimple.Direction.REVERSE);
        dcMotorMFL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void teleOpControls() {
        dcMotorMFL.setPower((ly + lx + rx) / max);
        dcMotorMBL.setPower((ly - lx + rx) / max);
        dcMotorMFR.setPower((ly - lx - rx) / max);
        dcMotorMBR.setPower((ly + lx - rx) / max);
    }
}
