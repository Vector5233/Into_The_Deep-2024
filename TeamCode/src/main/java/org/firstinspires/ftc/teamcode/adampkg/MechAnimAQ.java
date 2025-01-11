
package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//@Disabled
@TeleOp(group = "Qureshi", name = "ManualTeleop")
public class MechAnimAQ




        extends LinearOpMode {
    final RobotBase robotBase = new RobotBase();


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            servoTelemetry();
            teleOpControls();
        }

    }
    public void Lift()
    {
        if(gamepad1.y)
        {
            robotBase.liftDirection = 1;
        }
        else if(gamepad1.a)
        {
            robotBase.liftDirection = -1;
        }
        else {
            robotBase.liftDirection = 0;
        }
        robotBase.liftRight.setPower(robotBase.liftDirection);
        robotBase.liftLeft.setPower(robotBase.liftDirection);

        //return liftDirection;
    }
    public void initHardware() {
        initDrive();
        robotBase.initServos(hardwareMap);
    }

    private void initDrive() {
        robotBase.frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        robotBase.frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        robotBase.backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        robotBase.backRight = hardwareMap.get(DcMotor.class, "rightBack");
        robotBase.liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        robotBase.liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        robotBase.liftLeft.setDirection(DcMotor.Direction.REVERSE);
        //back left reverse os up for debate because we changed hardware and it stopped working
        robotBase.backLeft.setDirection(DcMotor.Direction.REVERSE);
        robotBase.frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void ServoMovement()
    {
        ServoPincher();
        PincherPivot();
        robotBase.Extension(gamepad2);
    }

    private void PincherPivot() {
        if(gamepad1.right_trigger >= 0.5)
        {
            robotBase.pincherPivot.setPosition(robotBase.pincherPivotUp);
        }
        if(gamepad1.left_trigger >= 0.5)
        {
            robotBase.pincherPivot.setPosition(robotBase.pincherPivotDown);
        }
    }


    private void ServoPincher() {
        if(gamepad1.right_bumper)
        {
            robotBase.servoPincher.setPosition(robotBase.servoPincherPositionClosed);
        }
        if(gamepad1.left_bumper)
        {
            robotBase.servoPincher.setPosition(robotBase.servoPincherPositionOpena);
        }
    }

    public void driveTrain()
    {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        robotBase.frontLeft.setPower((ly + lx + rx) / max);
        robotBase.frontRight.setPower((ly - lx - rx) / max);
        robotBase.backLeft.setPower((ly - lx + rx) / max);
        robotBase.backRight.setPower((ly + lx - rx) / max);
    }
    public void servoTelemetry() {
        //telemetry.log().clear();
        telemetry.addData("Position", robotBase.servoPincher.getPosition());
        telemetry.addData("Direction", robotBase.servoPincher.getDirection());
        telemetry.addData("Controller", robotBase.servoPincher.getController());

        telemetry.addData("Position", robotBase.pincherPivot.getPosition());
        telemetry.addData("Direction", robotBase.pincherPivot.getDirection());
        telemetry.addData("Controller", robotBase.pincherPivot.getController());

        telemetry.addData("Position", robotBase.pincherPivot.getPosition());
        telemetry.addData("Direction", robotBase.pincherPivot.getDirection());
        telemetry.addData("Controller", robotBase.pincherPivot.getController());
    }
    public void teleOpControls() {
        driveTrain();
        ServoMovement();
        Lift();
        robotBase.liftDirection = 0;
    }
}
