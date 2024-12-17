package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//@Disabled
@Autonomous(name="AutoLiftTest", group="VECTORAUTO")

public class AutoLiftTest extends LinearOpMode {
    DcMotor liftLeft;
    DcMotor liftRight;
    int liftPosUp = 100;
    int liftPosDown = -300;
    double liftPower = 1.0;
    double liftZero = 0;
    int positionTolerance = 10; // Tolerance for encoder value

    @Override
    public void runOpMode() throws InterruptedException {
        inits();
        waitForStart();

        liftTelemetry();
        while (!opModeIsActive() && !isStopRequested()) {
            liftTelemetry();
        }

        if (opModeIsActive()) {
            runLiftToPosition(liftPosUp);
            sleep(2000);
        }

        sleep(5000);
        requestOpModeStop();
    }

    public void inits() {
        initLifts();
    }

    public void initLifts() {
        liftRight = hardwareMap.get(DcMotor.class, "rightLift");
        liftLeft = hardwareMap.get(DcMotor.class, "leftLift");

        liftLeft.setDirection(DcMotor.Direction.REVERSE);
        liftLeft.setPower(liftZero);
        liftRight.setPower(liftZero);

        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runLiftToPosition(int position) {
        liftLeft.setTargetPosition(position);
        liftRight.setTargetPosition(position);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);

        // Loop until the motors are within the tolerance range or the op mode stops
        while (opModeIsActive() &&
                (Math.abs(liftLeft.getCurrentPosition() - position) > positionTolerance ||
                        Math.abs(liftRight.getCurrentPosition() - position) > positionTolerance)) {
            liftTelemetry();
        }

        // Stop the motors after reaching the target
        liftLeft.setPower(0);
        liftRight.setPower(0);

        // Set motors back to RUN_USING_ENCODER for other operations
        // liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /// liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void liftTelemetry() {
        telemetry.addData("LiftLeft", "Encoder: %2d, Power: %2f", -liftLeft.getCurrentPosition(), liftLeft.getPower());
        telemetry.addData("LiftRight", "Encoder: %2d, Power: %2f", -liftRight.getCurrentPosition(), liftRight.getPower());
        telemetry.update();
    }
}
