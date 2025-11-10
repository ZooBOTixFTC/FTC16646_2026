package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kMaxVel = 0.75;
        public static final double kMidFieldVel = 0.6;

        public static final double kP = 2;
        public static final double kFLeft = 1.1;
        public static final double kFRight = 1;
    }

    public static final class IntakeConstants {
        public static final double kIntakeOn = 1;
        public static final double kIntakeAuto = 0.75;
        public static final double kIntakeOff = 0;
        public static final double kIntakeReverse = -1;
    }

    public static final class LiftConstants {
        public static final double kTicksPerRev = 3895.9; // 139:1 GoBilda motor

        public static final int kLift = 50;
        public static final int kHome = 0;
        public static final int kKick = -20;//-15
    }

    public static final class AutoAlignConstants{
        public static final double kP = 0.01;
        public static final double kD = 0.0;
        public static final double kTurn = .07;
        public static final double kDrive = -.03;
        public static final double kMinTurn = .1;
        public static final double kMaxTurn = .15;
        public static final double kCloseTolerance = 2; // deg
        public static final double kFarTolerance = 2; // deg
        public static final double kDistanceThreshold = 90; // close vs. far, in.
    }

    public static class SystemConstants{
        public static final int kSysUpdateRateHz = 10;
    }
}


