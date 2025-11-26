package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 640;
        public static final double kAlpha = 0.1;

        public static final double kP = 0.0001;
        public static final double kD = 0.000007;
        public static final double kV = 0.0000292;
        public static final double kS = 0.06;
        public static final double kKickPosition = 0.65;
        public static final double kKickHome = 0.5;
        public static final double kKickDuration = 85;
        public static final double kTicksPerRev = 2077; // motor PPR × gear ratio old ratio 537.7 * (331 / 64.0)
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -1;
    }

    public static final class ColorConstants {
        public static final double kOrange = .333;
        public static final double kGreen = .45;
        public static final double kPurple = .722;

        public enum ColorEnum {
            GREEN
            ,PURPLE
            ,UNKNOWN
        }
    }
}


