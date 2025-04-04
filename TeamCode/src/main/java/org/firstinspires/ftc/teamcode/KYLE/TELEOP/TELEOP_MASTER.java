
package org.firstinspires.ftc.teamcode.KYLE.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(group = "Qureshi", name = "StatesTeleop")
public class TELEOP_MASTER extends LinearOpMode {
    final RobotBase robotBase = new RobotBase();

    boolean slowMode = false;
    boolean slowModeDebounce = false;
    int liftTicks;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            servoTelemetry();
            teleOpControls();
            telemetry.update();
        }

    }
    public void runLiftsToPos(int position)
    {
        robotBase.initServos(hardwareMap);
        robotBase.liftLeft.setTargetPosition(position);
        robotBase.liftRight.setTargetPosition(position);
        robotBase.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBase.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Lift()
    {
        if(gamepad1.y)
        {
            robotBase.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotBase.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotBase.liftDirection = 1;
        }
        else if(gamepad1.a)
        {
            robotBase.liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotBase.liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robotBase.liftDirection = -1;
        }
        else {
            liftTicks = robotBase.liftLeft.getCurrentPosition();
            robotBase.liftDirection = 0;
            runLiftsToPos(liftTicks);
        }
        robotBase.liftRight.setPower(robotBase.liftDirection);
        robotBase.liftLeft.setPower(robotBase.liftDirection);

        //return liftDirection;
    }
    public void initHardware() {
        // moved hardwareMaps to robot base
        robotBase.initServos(hardwareMap);
        robotBase.initDrive(hardwareMap);
    }

    public void ServoMovement()
    {
        ServoPincher();
        PincherPivot();
        robotBase.Extension(gamepad2);
    }

    private void PincherPivot() {
        if(gamepad1.right_trigger >= 0.5 || (gamepad2.right_trigger >= 0.5))
        {
            robotBase.pincherPivot.setPosition(robotBase.pincherPivotUp);
        }
        if(gamepad1.left_trigger >= 0.5 || gamepad2.left_trigger >= 0.5)
        {
            robotBase.pincherPivot.setPosition(robotBase.pincherPivotDown);
        }
    }


    private void ServoPincher() {
        if(gamepad1.right_bumper||gamepad2.right_bumper)
        {
            robotBase.servoPincher.setPosition(robotBase.servoPincherPositionClosed);
        }
        if(gamepad1.left_bumper||gamepad2.left_bumper)
        {
            robotBase.servoPincher.setPosition(robotBase.servoPincherPositionOpen);
        }
    }
    public void slowModeSwitcher()
    {
        if(gamepad1.left_stick_button){
            if(!slowModeDebounce){
                slowMode = !slowMode;
                slowModeDebounce = true;
                telemetry.addData("stick", "stick down");
            }
        }else{
            slowModeDebounce = false;
            telemetry.addData("stick", "stick up");
        }
        telemetry.update();
    }
    public void driveTrain()
    {
        double speedMult = slowMode ? 0.5 : 1;

        double lx = -gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
        robotBase.frontLeft.setPower(((ly + lx + rx) / max)*speedMult);
        robotBase.frontRight.setPower(((ly - lx - rx) / max)*speedMult);
        robotBase.backLeft.setPower(((ly - lx + rx) / max)*speedMult);
        robotBase.backRight.setPower(((ly + lx - rx) / max)*speedMult);
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
        slowModeSwitcher();
        driveTrain();
        ServoMovement();
        Lift();
        robotBase.liftDirection = 0;

    }
}
