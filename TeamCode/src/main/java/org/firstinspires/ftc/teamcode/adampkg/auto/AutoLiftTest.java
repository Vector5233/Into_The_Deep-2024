package org.firstinspires.ftc.teamcode.adampkg.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//@Disabled
@Autonomous(name="AutoLiftTest", group="VECTORAUTO")

public class AutoLiftTest extends LinearOpMode {
    private DcMotor liftLeft;
    private DcMotor liftRight;

    public int liftPosUp = 300;
    public int liftPosDown = -300;

    double liftRightPower = 1.0;
    double liftLeftPower = 1.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        //do Initializations
        initLifts();

        liftLeft.setTargetPosition(liftPosUp);
        liftRight.setTargetPosition(liftPosUp);
        while(opModeIsActive() && liftLeft.isBusy())
        {
            telemetry.addData("liftLeft Currently At: %7d", liftLeft.getCurrentPosition());
            telemetry.addData("liftRight Currently At: %7d", liftRight.getCurrentPosition());
            telemetry.addData("Running to:", "%7d", liftPosUp);
            telemetry.update();
            sleep(1000);
        }
        liftLeft.setTargetPosition(liftPosDown);
        liftRight.setTargetPosition(liftPosDown);
        while(opModeIsActive() && liftLeft.isBusy())
        {
            telemetry.addData("liftLeft Currently At: %7d", liftLeft.getCurrentPosition());
            telemetry.addData("liftRight Currently At: %7d", liftRight.getCurrentPosition());
            telemetry.addData("Running to:", "%7d", liftPosUp);
            telemetry.update();
            sleep(1000);
        }


    }

    public void initLifts()
    {
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");
        liftLeft.setDirection(DcMotor.Direction.REVERSE);

        liftLeft.setPower(liftLeftPower);
        liftRight.setPower(liftRightPower);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);



        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}
