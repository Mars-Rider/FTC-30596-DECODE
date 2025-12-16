package org.firstinspires.ftc.teamcode.Robot.LEDs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.LEDs.LEDController.*;

@TeleOp(name = "Sample", group = "LEDs")
public class Example extends LinearOpMode{
    VoltageSensor voltageSensor;
    double voltMax = 13;
    double voltMin = 9;

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //Makes new ledController
        LEDController ledController = new LEDController(hardwareMap,"LEDs");

        //Adds a strip to the led controller
        LEDChannel strip1 = ledController.addChannel(); //For Voltage - Adds a strip to index 1
        LEDChannel strip2 = ledController.addChannel(); //For Underglow :) - Adds a strip to index 2
        LEDChannel strip3 = ledController.addChannel(); //For State of Sensor/Controller - Adds a strip to index 3, etc.

        strip1.color.setValueRange(voltMin,voltMax,130, 0);
        strip1.pattern.setScanner(10, true);

        strip2.color.setRainbow();
        strip2.pattern.setBreathe();

        strip3.color.setOrange();
        strip3.pattern.setSolid();

        waitForStart();
        //On Start
        strip1.color.setGreen();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currVolt = voltageSensor.getVoltage();

            strip1.color.updateValue(currVolt);

            if(currVolt < .5 + voltMin){
                strip1.pattern.setBlink();
            } else {
                strip1.pattern.setSolid();
            }

            if(gamepad1.a){
                strip3.color.setBlue();
            } else {
                strip3.color.setPink();
            }
        }
    }
}