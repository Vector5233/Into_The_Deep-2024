package org.firstinspires.ftc.teamcode.training;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*resources and tips
    RevBlinkin driver user Manual - print out chrome-extension://efaidnbmnnnibpcajpcglclefindmkaj/https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
    YouTube video: http://youtu.be/wMdkM2rr1a4
    5V addressable Leds- on rev robotics website.
    Use two Blinkin Controllers if possible they like to change modes.
    In oder to change the mode back, place the module in an accessible location on the bot.
 */

/**
 * Configuration file
 * Control Hub
 * Servo port 05:leftLights
 * Expansion Hub
 * Servo port 05:rightLights
 */

@Disabled
@TeleOp(group = "Primary", name = "Led Controller")
public class Example_004d_BlinkinLEDController extends LinearOpMode {
    private RevBlinkinLedDriver leftLights, rightLights;
    boolean blinkinTimer = false;
    int blinkinDelay = 2000;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            timeTelemetry();
        }
        waitForStart();
        resetBlinkin();
        while (opModeIsActive()) {
            updateBlinkin();
        }
    }

    public void initHardware() {
        initLights();
        sleep(blinkinDelay);
        blinkinGreen();

    }

    public void initLights() {
        leftLights = hardwareMap.get(RevBlinkinLedDriver.class, "leftLights");
        rightLights = hardwareMap.get(RevBlinkinLedDriver.class, "rightLights");
    }

    public void resetBlinkin() {
        blinkinTimer = true;
        resetRuntime();
    }

    public void updateBlinkin() {
        if (blinkinTimer && time < 5) {
            blinkinRed();
        } else if(blinkinTimer && time >= 5 && time < 10)
        {
            blinkinGreen();

        }
         else if(blinkinTimer && time >= 10 && time < 15)
        {
            blinkinBlue();

        }
         else if(blinkinTimer && time >= 15 && time < 20)
        {
            blinkinOrange();

        }
        else{
            blinkinBlack();
        }
    }
    public void blinkinRed(){
        leftLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        rightLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

    }
    public void blinkinGreen(){
        leftLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        rightLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

    }
    public void blinkinBlue(){
        leftLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        rightLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

    }
    public void blinkinOrange(){
        leftLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        rightLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);

    }
    public void blinkinBlack(){
        leftLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        rightLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

    }

    public void timeTelemetry() {
    telemetry.addData("Timer", time);
    telemetry.update();

    }
}
