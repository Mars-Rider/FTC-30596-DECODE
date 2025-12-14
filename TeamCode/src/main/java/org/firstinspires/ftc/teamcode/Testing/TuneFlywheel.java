package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Settings;

@TeleOp(name = "Flywheel Velocity Tuning", group = "Tests")
public class TuneFlywheel extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        double oldkP = Settings.FlyWheel.kP, oldkI = Settings.FlyWheel.kI, oldkD = Settings.FlyWheel.kD, oldkF = Settings.FlyWheel.kF;

        waitForStart();
        //On Start

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(Settings.FlyWheel.kP != oldkP || Settings.FlyWheel.kI != oldkI || Settings.FlyWheel.kD != oldkD || Settings.FlyWheel.kF != oldkF){
                oldkP = Settings.FlyWheel.kP;
                oldkI = Settings.FlyWheel.kI;
                oldkD = Settings.FlyWheel.kD;
                oldkF = Settings.FlyWheel.kF;

                robot.bFly.setVelocityPIDFCoefficients(Settings.FlyWheel.kP, Settings.FlyWheel.kI, Settings.FlyWheel.kD, Settings.FlyWheel.kF);
            }

            if(gamepad1.aWasPressed()){
                robot.flyPower();
            }

            telemetry.addLine("Error: " + (robot.bFly.getVelocity(AngleUnit.RADIANS)-robot.flySpeed));
            telemetry.addLine("kP: " + Settings.FlyWheel.kP);
            telemetry.addLine("kI: " + Settings.FlyWheel.kI);
            telemetry.addLine("kD: " + Settings.FlyWheel.kD);
            telemetry.addLine("kF: " + Settings.FlyWheel.kF);
            telemetry.update();
        }
    }
}