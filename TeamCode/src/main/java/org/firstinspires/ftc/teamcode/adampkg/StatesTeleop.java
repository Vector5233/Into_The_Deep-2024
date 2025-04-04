
package org.firstinspires.ftc.teamcode.adampkg;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//@Disabled
@TeleOp(group = "Qureshi", name = "StatesTeleop")
public class StatesTeleop extends LinearOpMode {
    int MAX_HEIGHT = 6130;
    int SCORING_HEIGHT = 2575;
    int DOWN_SCORING_HEIGHT = 1850;
    final RobotBase robotBase = new RobotBase();

    boolean slowMode = false;
    boolean slowModeDebounce = false;

    boolean liftTicksDebounce = false;
    int liftTicks;

    boolean dpadDebounce = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
        }
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Touch Sensor:", robotBase.touchSensor.getValue());
            tickReset();
            servoTelemetry();
            teleOpControls();
            telemetry.update();
        }

    }
    public void tickReset()
    {
        if(robotBase.touchSensor.isPressed())
        {
            initMotors();
            telemetry.addData("Is pressed", robotBase.touchSensor.isPressed());
            liftTicks = 0;
        }
    }
    public void initMotors()
    {
        robotBase.liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robotBase.liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runLiftsToPos(int position)
    {
      //  robotBase.initServos(hardwareMap);
        robotBase.liftLeft.setTargetPosition(position);
        robotBase.liftRight.setTargetPosition(position);
        robotBase.liftLeft.setPower(1);
        robotBase.liftRight.setPower(1);
        robotBase.liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robotBase.liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void  motorTelemetry(){
        telemetry.addData("leftLift","Encoder: %2d, Power: %2f", robotBase.liftLeft.getCurrentPosition(), robotBase.liftLeft.getPower());
        telemetry.addData("leftRight","Encoder: %2d, Power: %2f", robotBase.liftRight.getCurrentPosition(), robotBase.liftRight.getPower());
        telemetry.update();
    }
    public void waitLifts(int holdTime)
    {
        while(opModeIsActive() && robotBase.liftRight.isBusy() && robotBase.liftLeft.isBusy())
        {
            motorTelemetry();
        }
        sleep(holdTime);
    }
    public void Lift()
    {
        telemetry.addData("Left Tick number:", robotBase.liftLeft.getCurrentPosition());
        telemetry.addData("Right Tick number:", robotBase.liftRight.getCurrentPosition());

        if (gamepad1.dpad_left)
        {
            if(!dpadDebounce) {
                runLiftsToPos(SCORING_HEIGHT);
                dpadDebounce = true;
            }

        }
        else if (gamepad1.dpad_up)
        {
            if(!dpadDebounce) {
                runLiftsToPos(MAX_HEIGHT);
                dpadDebounce = true;
            }
        }
        else if (gamepad1.dpad_down) {
            if(!dpadDebounce) {
                runLiftsToPos(0);
                dpadDebounce = true;
            }
        }
        else if (gamepad1.dpad_right){
            if(!dpadDebounce) {
                runLiftsToPos(DOWN_SCORING_HEIGHT);
                dpadDebounce = true;
            }
        } else {
            dpadDebounce = false;
        }

        if(gamepad1.y)
        {
            liftTicksDebounce = false;
            robotBase.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robotBase.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robotBase.liftDirection = 1;
        }
        else if(gamepad1.a)
        {
            liftTicksDebounce = false;
            robotBase.liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robotBase.liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robotBase.liftDirection = -1;
        }
        else {
            if(!liftTicksDebounce){
                liftTicks = robotBase.liftLeft.getCurrentPosition();
                liftTicksDebounce = true;
                runLiftsToPos(liftTicks);
            }
            robotBase.liftDirection = 0.5;
          //  runLiftsToPos(liftTicks);
        }
        robotBase.liftRight.setPower(robotBase.liftDirection);
        robotBase.liftLeft.setPower(robotBase.liftDirection);
        telemetry.addData("liftTicks", liftTicks);


        //return liftDirection;
    }
    public void initHardware() {
        initDrive();
        initMotors();
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
            robotBase.servoPincher.setPosition(robotBase.servoPincherPositionOpena);
        }
    }
    public void slowModeSwitcher()
    {
        if(gamepad1.left_stick_button){
            if(!slowModeDebounce){
                slowMode = !slowMode;
                slowModeDebounce = true;
            }
        }else{
            slowModeDebounce = false;
         //   telemetry.addData("stick", "stick up");
        }
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
