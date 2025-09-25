package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = true;
        public static final double kTolerance = 10;
        public static final double kShooterP = 0;
        public static final double kShooterD = 0;
        public static final double kShooterF = 0;
        public static final double kickOn = 1;
        public static final double kickOff = 0;
        public static final double kickDuration = 0.5;

        public static final double[][] kShooterInterpolationTable = {
                {36, 125},
                {48, 150},
                {60, 175},
                {72, 200},
                {84, 225},
                {96, 250},
                {108, 275},
                {120, 300}
        };
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


