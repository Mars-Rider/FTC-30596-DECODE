package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp(name = "LEDTest", group = "Tests")
public class LEDTest extends LinearOpMode {

    I2cDeviceSynch led;

    @Override
    public void runOpMode() throws InterruptedException {
        led = hardwareMap.get(I2cDeviceSynch.class, "ledController");
        led.engage();   // enable communication

        waitForStart();

        while (opModeIsActive()) {
            // Example: Set color to purple
            sendCommand("SET 128 0 255");

            sleep(200);
        }
    }

    private void sendCommand(String s) {
        byte[] data = s.getBytes();
        led.write(0x00, data);
    }
}

