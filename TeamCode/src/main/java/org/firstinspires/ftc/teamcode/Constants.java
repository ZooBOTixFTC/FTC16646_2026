package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = true;
        public static final double kTolerance = 750;
        public static final double kShooterP = 0.005;
        public static final double kShooterD = 0;
        public static final double kShooterFarF = 0.76;
        public static final double kShooterMidF = 0.55;
        public static final double kMaxVelDegPerSec = 24000;
        public static final double kMidFieldDegPerSec = 18500;
        public static final double kKickPosition = 0.8;
        public static final double kKickHome = 0.55;
        public static final double kKickDuration = 0.5;
        public static final double kTicksPerRev = 2075; // motor PPR × gear ratio old ratio 537.7 * (331 / 64.0)
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


