package org.firstinspires.ftc.teamcode.Robot.LEDs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Robot.LEDs.LEDController.*;

@TeleOp
public class Example extends LinearOpMode{
    VoltageSensor voltageSensor;
    double voltMax = 13;
    double voltMin = 9;

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        LEDController ledController = new LEDController(hardwareMap,"LEDs");
        LEDChannel strip1 = ledController.addChannel("voltage");
        LEDChannel strip2 = ledController.addChannel("underglow");

        strip2.setPattern(Pattern.RAINBOW);

        strip1.setColor(Color.ORANGE);
        strip1.setPattern(Pattern.SCANNER);

        waitForStart();
        //On Start
        strip1.setColor(Color.GREEN);
        strip1.setPattern(Pattern.SOLID);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currVolt = voltageSensor.getVoltage();

            strip1.setHSL(Math.min(120,Math.max(0,((currVolt/voltMax)-voltMin)*120)),255,255);

            if(currVolt < .5 + voltMin){
                strip1.setPattern(Pattern.BLINK);
            }

        }
    }
}