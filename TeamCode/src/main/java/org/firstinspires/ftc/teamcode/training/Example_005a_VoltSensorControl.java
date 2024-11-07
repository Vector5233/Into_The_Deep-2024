package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Configuration file
 */

@Disabled
@TeleOp(group = "Primary", name = "VoltTester")
public class Example_005a_VoltSensorControl extends LinearOpMode {
    private VoltageSensor voltageSensor;
    int voltSensorThreshold = 11;
    private DcMotor motorOne;
    double motorOneZeroPower = 0.0;
    double motorOnePower = 1.0;


    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            voltageSensorTelementry();
        }
        waitForStart();
        while (opModeIsActive()) {
            voltageSensorTelementry();
            teleOpControls();
        }
    }

    public void initHardware() {
        voltSensor();
        initMotorOne();
    }
    public  void  initMotorOne(){
        motorOne = hardwareMap.get(DcMotor.class, "motorOne");
        motorOne.setDirection(DcMotor.Direction.FORWARD);
        motorOne.setPower(motorOneZeroPower);
        motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//FLOAT for cost to a stop
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    public void  voltSensor(){
        voltageSensor =hardwareMap.voltageSensor.iterator().next();
    }
    public void teleOpControls(){
        if(gamepad2.x){
            motorOne.setPower(-motorOnePower);
        }
        if(gamepad2.a){
            motorOne.setPower(motorOneZeroPower);
        }
        if(gamepad2.b){
            motorOne.setPower(motorOnePower);
        }
        if((motorOne.getPower()==motorOnePower)&&(voltageSensor.getVoltage()== voltSensorThreshold)){
            motorOne.setPower(-motorOnePower);
        }
    }
    public void voltageSensorTelementry(){
        telemetry.addData("Voltage Sensor", voltageSensor.getVoltage());
        telemetry.update();
    }
}
