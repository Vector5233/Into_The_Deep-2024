package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**Configuration file
 * Motor port 1 : leftFM
 * Motor port 2 : rightFM
 * Motor port 3 : leftBM
 * Motor port 4 : rightBM
 */

@Disabled
@TeleOp(group = "Primary", name = "Short Name")
public class FieldCentricDrive extends LinearOpMode {
    private DcMotor leftFM;
    private DcMotor rightFM;
    private DcMotor leftBM;
    private DcMotor rightBM;

    double lx =gamepad1.left_stick_x;
    double ly = gamepad1.left_stick_y;
    double rx = gamepad1.right_stick_x;
    @Override
    public void  runOpMode() throws InterruptedException{
        initHardware();
        while(!isStarted()){}
        waitForStart();
        while(opModeIsActive()){

            double max = Math.max(Math.abs(lx)+ Math.abs(ly) + Math.abs(rx), 1);
            double power = 0.2 + (0.6 * gamepad1.right_trigger);

        leftFM.setPower(((ly+lx+rx)/max)* power);
        leftBM.setPower(((ly-lx+rx)/max)* power);
        rightFM.setPower(((ly-lx-rx)/max)* power);
        rightBM.setPower(((ly+lx-rx)/max)* power);



        }
    }
    public void initHardware(){
        initDriveMotors();

    }
    public void initDriveMotors(){
        leftFM = hardwareMap.get(DcMotor.class,"leftFM");
        rightFM = hardwareMap.get(DcMotor.class,"rightFM");
        leftBM = hardwareMap.get(DcMotor.class,"leftBM");
        rightBM = hardwareMap.get(DcMotor.class,"rightBM");

        // set oneside to reverse
        leftFM.setDirection(DcMotor.Direction.REVERSE);
        leftBM.setDirection(DcMotor.Direction.REVERSE);


    }
}
