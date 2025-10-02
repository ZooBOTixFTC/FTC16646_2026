package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = true;
        public static final double kTolerance = 1000;
        public static final double kShooterP = 0;
        public static final double kShooterD = 0;
        public static final double kShooterF = 0;
        public static final double kKickOn = 1;
        public static final double kKickOff = 0;
        public static final double kKickReverse = -1;
        public static final double kickDuration = 2;
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


