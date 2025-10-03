package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 250;
        public static final double kShooterP = 0.005;
        public static final double kShooterD = 0;
        public static final double kShooterF = 0.76;
        public static final double kKickOn = 1;
        public static final double kKickOff = 0;
        public static final double kKickReverse = -1;
        public static final double kKickDuration = 4;
        public static final double kFeedPowerOn = 1;
        public static final double kFeedPowerOff = 0;
        public static final double kMaxVelDegPerSec = 24000;
        public static final double kMidFieldDegPerSec = 22000;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -1;
    }

    public static final class LiftContsants {
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


