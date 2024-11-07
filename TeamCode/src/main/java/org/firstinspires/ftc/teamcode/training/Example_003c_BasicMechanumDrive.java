package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Configuration file
 * port 00: leftFront
 * port 01: leftBack
 * port 02: rightFront
 * port 03: rightBack
 */

//@Disabled
@TeleOp(group = "Primary", name = "Tank Mechanum Drive")
public class Example_003c_BasicMechanumDrive extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            // this drives the motors but setting the power lx to -1-0-1  forward and backward on Left joyStick and ly -1-0-1 left and right.
            // right joyStick spins the bot in rx left or right on center axis.
            teleOpControls();

        }
    }

    public void initHardware() {

        initDrive();
    }

    public void initDrive() {
        initiDriveMotors();
    }

    private void initiDriveMotors() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        // motor reversed as they are set to spin rotation right.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        //leftBack.setDirection(DcMotor.Direction.FORWARD);
    }

    public void teleOpControls() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double power = 0.8 + (0.3 * gamepad1.right_trigger);
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        leftFront.setPower((ly + lx + rx) / max * power);
        leftBack.setPower((ly - lx + rx) / max * power);
        rightFront.setPower((ly - lx - rx) / max * power);
        rightBack.setPower((ly + lx - rx) / max * power);
    }


}
