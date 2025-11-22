package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Settings {
    public static double sortMid = 0.5;
    public static double sortInc = 0.2;
    //public static double flySpeedIncre = 50;
    public static double flySpeedIncre = 0.05;

    public static class powerEstimate {
        public static double flyXOffset = 4;
        public static double flyYOffset = 12;
        public static double targetXOffset = 6;
        public static double targetYOffset = 6;
        public static double flyEfficency = 0.5;
        public static double flyAngle = 55;
        public static double maxRPM = 6000;
        public static double flywheelDiameter = 2.5;
    }

    public static double goalKp = 0;
    public static double goalDeadzone = 0.5;
}
