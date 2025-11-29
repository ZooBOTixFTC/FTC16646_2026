package org.firstinspires.ftc.teamcode;


//importing all of the hardwere etc
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;


//gives name and sets as lineropmode


@TeleOp(name ="Teleop")
public class Teleop extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        //init
        // Initialize drivetrain
        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap);

//gives all parts instructions on what to do (fact check this)

        Servo m_kicker = hardwareMap.get(Servo.class, "kicker");
        m_kicker.setDirection(Servo.Direction.REVERSE);

        DcMotor m_shooterMotorLeft = hardwareMap.get(DcMotor.class, "shooterMotorLeft");
        m_shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        m_shooterMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooterMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_shooterMotorLeft.setPower(0);

        DcMotor m_shooterMotorRight = hardwareMap.get(DcMotor.class, "shooterMotorRight");
        m_shooterMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        m_shooterMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_shooterMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_shooterMotorRight.setPower(0);

        DcMotor m_intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        m_intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        m_intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_intakeMotor.setPower(0);

        DcMotor m_beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
        m_beltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        m_beltMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_beltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_beltMotor.setPower(0);





        //init loop
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.update();

        }

        //this code is telling the moters and servos what to do when a button is pressed
        //enabled loop
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();

            // Drive controls
            double y = -gamepad1.left_stick_y;  // forward/back
            double x = gamepad1.left_stick_x;   // strafe
            double rx = gamepad1.right_stick_x; // rotation/


            drive.drive(y, x, rx);


            if (gamepad1.rightBumperWasPressed()) {
                m_shooterMotorLeft.setPower(0.9);
                m_shooterMotorRight.setPower(0.9);
                m_beltMotor.setPower(0.9);
                m_kicker.setPosition(0.0);

            }


            if (gamepad1.rightBumperWasReleased()) {
                m_shooterMotorLeft.setPower(0);
                m_shooterMotorRight.setPower(0);
                m_beltMotor.setPower(0);
                m_kicker.setPosition(0.5);


            }

            if (gamepad1.leftBumperWasPressed()) {
                m_intakeMotor.setPower(1);
                m_beltMotor.setPower(1);
            }

            if (gamepad1.leftBumperWasReleased()) {
                m_intakeMotor.setPower(0);
                m_beltMotor.setPower(0);
            }
            if (gamepad1.xWasPressed()) {
                m_intakeMotor.setPower(-0.5);
                m_beltMotor.setPower(-0.5);
            }
            if (gamepad1.xWasReleased()){
                m_intakeMotor.setPower(0);
                m_beltMotor.setPower(0);














            }

        }

    }
}