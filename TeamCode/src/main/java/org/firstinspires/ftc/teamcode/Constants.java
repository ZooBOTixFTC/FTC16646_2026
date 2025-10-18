package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 250;
        public static final double kKick = 75;
        public static final double kKickHome = 0;
        public static final double kKickDuration = 2;
        public static final double kFeedOn = 1;
        public static final double kFeedOff = 0;

        public static final double kShooterFarP = 0.15;//.37 @ 22k
        public static final double kShooterD = 0;
        public static final double kShooterFarF = 0.5;//.325 @22k
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


