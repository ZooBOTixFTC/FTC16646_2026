package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 640;
        public static final double kKickRight = 55;
        public static final double kKickHomeRight = 5;
        public static final double kKickLeft = 45;
        public static final double kKickHomeLeft = 0;
        public static final double kKickDuration = 500;
        public static final double kAlpha = 0.1;

        public static final double kPLeft = 0;
        public static final double kVLeft = 0;
        public static final double kSLeft = 0;
        public static final double kPRight = 0;
        public static final double kVRight = 0;
        public static final double kSRight = 0;
        public static final double kMaxVelDegPerSec = 21000;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -1;
    }

    public static final class LiftConstants {
        public static final int kLiftUp = 100;
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


