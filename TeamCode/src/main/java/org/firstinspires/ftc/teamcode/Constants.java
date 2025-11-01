package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 50;
        public static final double kAlpha = 0.1;

        public static final double kPLeft = 0.0005;
        public static final double kDLeft = 0.0;
        public static final double kVLeft = 0.00019;
        public static final double kSLeft = 0.0;

        public static final double kPRight = 0.0005;
        public static final double kDRight = 0.0;
        public static final double kSRight = 0.0;
        public static final double kVRight = 0.00019;

        public static final double kMaxVel = 4000;
        public static final double kMaxAccel = 1000;
        public static final double kMidFieldVel = 3500;
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
        public static final int kKick = -14;
    }
}


