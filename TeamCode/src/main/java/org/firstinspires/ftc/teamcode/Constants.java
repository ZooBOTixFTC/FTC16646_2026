package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = true;
        public static final double kTolerance = 640;
        public static final double kAlpha = 0.1;

        public static final double kPLeft = 0.0001;
        public static final double kDLeft = 0.00001;
        public static final double kVLeft = 0.0000284;
        public static final double kSLeft = 0.037;
        public static final double kPRight = 0.0;
        public static final double kDRight = 0.0;
        public static final double kSRight = 0.046;
        public static final double kVRight = 0.0;
        public static final double kMaxVelDegPerSec = 24500;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -1;
    }

    public static final class LiftConstants {
        public static final double kTicksPerRev = 3895.9; // 139:1 GoBilda motor
        public static final double kKickDuration = 500;

        public static final int kLift = 10;
        public static final int kHome = 0;
        public static final int kKick = -10;
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


