package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kFarVel = 0.65;//.65 orange
        public static final double kCloseVel = .5;
        public static final double kMidFieldVel = 0.555;
        public static final double kPreRev = .45;
        public static final double kTolerance = 0.01;
        public static final double kMaxVel = 2600;
        public static final double kP = 10;
        public static final double kFLeft = 1.05;//1.02 orange 1.1
        public static final double kFRight = 1.0; //.98 orange 1.0 grey
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;
        public static final double kIntakeAuto = 0.75;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -.67;
    }

    public static final class LiftConstants {
        public static final double kTicksPerRev = 3895.9; // 139:1 GoBilda motor

        public static final int kLift = 35;
        public static final int kHome = 0;
        public static final int kKick = -22;
        public static final int kLevel = -18;
    }

    public static final class AutoAlignConstants{
        public static final double kMidRangeThreshold = 45;
        public static final double kDistanceThreshold = 90; // close vs. far, in.
    }

    public static class SystemConstants{
        public static final int kSysUpdateRateHz = 10;
    }
}


