package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Settings {
    public static double sortMid = 0.5;
    public static double sortInc = 0.2;
    public static double flySpeedIncre = 5;
    //public static double flySpeedIncre = 0.05;

    public static boolean flipFR = false;
    public static boolean flipFL = false;
    public static boolean flipBR = false
            ;
    public static boolean flipBL = false;



    public static class powerEstimate {
        public static double flyXOffset = 4;
        public static double flyYOffset = 12;
        public static double targetXOffset = 6;
        public static double targetYOffset = 6;
        public static double flyEfficency = 1;
        public static double flyOverEstimate = 0.625;
        public static double flyAngle = 55;
        public static double maxRPM = 6000;
        public static double flywheelDiameter = 2.5;
    }

    public static double goalKp = 0;
    public static double goalDeadzone = 0.5;

    public static class FlyWheel {
        public static double kP = 10;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 12.5;
    }
}
