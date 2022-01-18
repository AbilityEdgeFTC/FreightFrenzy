package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.Subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.Subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.intake;

import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.currentPose;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    DcMotor mC, mE, mFL, mBL, mFR, mBR, mI;
    Servo sD;

    public static double powerCarousel = 0.325;
    public static double powerIntake = 1;
    public static double intakePosition = 1, dippingPosition = .6;
    public static double lockOn = 90;
    public static double mainPower = 1;
    public static boolean isRegularDrive = true;

    public static double MAX_HEIGHT = 15.5; // TODO set value in inches
    public static double MID_HEIGHT = 9; // TODO set value in inches
    public static double MIN_HEIGHT = 4; // TODO set value in inches
    public static double ZERO_HEIGHT = 0; // TODO set value in inches
    public static boolean moveToMin = false;
    public static boolean moveToMid = false;
    public static boolean moveToMax = false;
    public static boolean moveToZero = false;
    public static double timeTo = 3;

    carousel carousel;
    Elevator elevator;
    intake intake;
    gamepad gamepads;
    dip dip;

    @Override
    public void runOpMode() throws InterruptedException {
        initAll();
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(currentPose);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cGamepad cGamepad1 = new cGamepad(gamepad1);
        cGamepad cGamepad2 = new cGamepad(gamepad2);

        elevator = new Elevator(hardwareMap, MAX_HEIGHT, MID_HEIGHT, MIN_HEIGHT, ZERO_HEIGHT);
        NanoClock clock = NanoClock.system();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        carousel = new carousel(mC, powerCarousel);
        gamepads = new gamepad(gamepad1, gamepad2, mFL, mBL, mFR, mBR, mainPower, isRegularDrive, telemetry, drive, lockOn);

        intake = new intake(mI, powerIntake);
        dip = new dip(sD, intakePosition, dippingPosition);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double startTime = clock.seconds();
            checkLevel();

            cGamepad1.update();
            cGamepad2.update();
            gamepads.update();

            if(gamepad1.right_stick_button || gamepad1.left_stick_button)
            {
                mainPower = .6;
            }
            else
            {
                mainPower = 1;
            }

            // TODO: change to gamepad2
            if(gamepad1.dpad_down)
            {
                dip.getFreight();
                moveToZero = true;
                moveToMin = false;
                moveToMid = false;
                moveToMax = false;
            }

            // TODO: change to gamepad1
            if(gamepad2.a)
            {
                gamepads.lockOnAngle = !gamepads.lockOnAngle;
            }

            goToPoistions(clock, startTime);

            if(moveToMin)
            {
                elevator.setHeight(Elevator.MIN_HEIGHT);
            }
            else if(moveToMid)
            {
                elevator.setHeight(Elevator.MID_HEIGHT);
            }
            else if(moveToMax)
            {
                elevator.setHeight(Elevator.MAX_HEIGHT);
            }
            else if(moveToZero)
            {
                elevator.setHeight(Elevator.ZERO_HEIGHT);
            }

            // TODO: change to gamepad2
            if (gamepad1.left_trigger != 0) {
                intake.powerIntake(-gamepad1.left_trigger);
            }
            // TODO: change to gamepad2
            else if (gamepad1.right_trigger != 0) {
                intake.powerIntake(gamepad1.right_trigger);
            } else {
                intake.stop();
            }

            // TODO: change to gamepad2
            if (/*cGamepad1.dpadRight() || cGamepad1.dpadLeft()*/ gamepad1.dpad_right || gamepad1.dpad_left) {
                carousel.spin();
            }
            else {
                carousel.stop();
            }
        }
    }

    void initAll() {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mI = hardwareMap.get(DcMotor.class, "mI");
        mI.setDirection(DcMotor.Direction.REVERSE);
        ;
        mC = hardwareMap.get(DcMotor.class, "mC");
        mE = hardwareMap.get(DcMotorEx.class, "mE");
        // use braking to slow the motor down faster
        mE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        mE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sD = hardwareMap.get(Servo.class, "sE");
    }

    void checkLevel()
    {
        if(gamepad1.a)
        {
            moveToMin = true;
            moveToMid = false;
            moveToMax = false;
            moveToZero = false;
        }
        else if(gamepad1.b)
        {
            moveToMid = true;
            moveToMin = false;
            moveToMax = false;
            moveToZero = false;
        }
        else if(gamepad1.y)
        {
            moveToMax = true;
            moveToMin = false;
            moveToMid = false;
            moveToZero = false;
        }
        else if(gamepad1.x)
        {
            moveToZero = true;
            moveToMin = false;
            moveToMid = false;
            moveToMax = false;
        }
    }

    void goToPoistions(NanoClock clock, double startTime)
    {
        if (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToMin)
        {
            elevator.update();
            telemetry.addData("targetVelocity", elevator.getTargetVelocity());
            telemetry.addData("measuredVelocity", elevator.getVelocity());
            telemetry.update();
            checkLevel();
        }
        else if (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToMid)
        {
            elevator.update();
            telemetry.addData("targetVelocity", elevator.getTargetVelocity());
            telemetry.addData("measuredVelocity", elevator.getVelocity());
            telemetry.update();
            checkLevel();
        }else if (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToMax)
        {
            elevator.update();
            telemetry.addData("targetVelocity", elevator.getTargetVelocity());
            telemetry.addData("measuredVelocity", elevator.getVelocity());
            telemetry.update();
            checkLevel();
        }else if (!isStopRequested() && (clock.seconds() - startTime) < timeTo && moveToZero)
        {
            elevator.update();
            telemetry.addData("targetVelocity", elevator.getTargetVelocity());
            telemetry.addData("measuredVelocity", elevator.getVelocity());
            telemetry.update();
            checkLevel();
        }
    }
}
