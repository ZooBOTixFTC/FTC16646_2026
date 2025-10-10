package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 250;
        public static final double kShooterD = 0;
        public static final double kShooterFarF = 0.325;
        public static final double kShooterFarP = 0.37;
        public static final double kShooterMidF = 0.55;
        public static final double kKickOn = 1;
        public static final double kKickOff = 0;
        public static final double kKickReverse = -1;
        public static final double kKickDuration = 1.5;
        public static final double kFeedPowerOn = 1;
        public static final double kFeedPowerOff = 0;
        public static final double kMaxVelDegPerSec = 22000;
        public static final double kMidFieldDegPerSec = 18500;
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


