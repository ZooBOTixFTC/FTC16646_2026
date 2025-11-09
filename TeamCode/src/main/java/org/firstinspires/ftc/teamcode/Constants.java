package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final class ShooterConstants {
        public static final boolean kTuningMode = false;
        public static final double kTolerance = 50;
        public static final double kAlpha = 0.1;

        public static final double kPLeft = 0.0001;
        public static final double kDLeft = 0.0;
        public static final double kSLeft = 0.047;
        public static final double kVLeft = 0.000176;

        public static final double kPRight = 0.0001;
        public static final double kDRight = 0.0;
        public static final double kSRight = 0.046;
        public static final double kVRight = 0.000175;

        public static final double kMaxVel = 4000;
        public static final double kMidFieldVel = 3500;
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


