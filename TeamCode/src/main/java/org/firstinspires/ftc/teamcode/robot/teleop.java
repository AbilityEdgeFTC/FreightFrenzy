package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.Subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.Subsystems.elevator;
import org.firstinspires.ftc.teamcode.robot.Subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.intake;

import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.currentPose;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    DcMotor mC, mE, mFL, mBL, mFR, mBR, mI;
    Servo sD;

    public static double powerCarousel = 0.325;
    public static double powerIntake = 1;
    public static double powerElevator = .5;
    public static int positionLevelOne = 170;
    public static int positionLevelTwo = 250;
    public static int positionLevelThree = 500;
    public static double intakePosition = 1, dippingPosition = .6;
    public static double lockOn = 90;
    public static double mainPower = .9;
    public static boolean isRegularDrive = true;

    carousel carousel;
    elevator elevator;
    intake intake;
    gamepad gamepads;
    dip dip;

    // TODO: TUNE
    public static double kP = 0, kI = 0, kD = 0;

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

        carousel = new carousel(mC, powerCarousel);
        elevator = new elevator(mE, powerElevator, kP, kI, kD, telemetry, positionLevelOne, positionLevelTwo, positionLevelThree);
        gamepads = new gamepad(gamepad1, gamepad2, mFL, mBL, mFR, mBR, mainPower, isRegularDrive, telemetry, drive, lockOn);

        intake = new intake(mI, powerIntake);
        dip = new dip(sD, intakePosition, dippingPosition);

        waitForStart();

        while (opModeIsActive()) {
            cGamepad1.update();
            cGamepad2.update();
            gamepads.update();
            gamepads.regularDrive = isRegularDrive;

            if(gamepad1.right_stick_button || gamepad1.left_stick_button)
            {
                mainPower = .4;
            }
            else
            {
                mainPower = .7;
            }

            // TODO: change to gamepad 1 right or left bumber
            if (cGamepad1.dpadUpOnce()) {
                dip.releaseFreight();
            }

            // TODO: change to gamepad2
            if(gamepad1.dpad_down)
            {
                dip.getFreight();
                elevator.goToZeroPos();
            }

            // TODO: change to gamepad1
            if(gamepad2.a)
            {
                gamepads.lockOnAngle = !gamepads.lockOnAngle;
            }

            // TODO: change to gamepad2
            // BUTTON Y
            if (cGamepad1.YOnce()) {
                elevator.goToLevelThree();
            }

            // TODO: change to gamepad2
            // BUTTON A
            if (cGamepad1.AOnce()) {
                elevator.goToLevelOne();
            }

            // TODO: change to gamepad2
            // BUTTON B
            if (cGamepad1.BOnce() || cGamepad1.XOnce()) {
                elevator.goToLevelTwo();
            }

            // TODO: change to gamepad2
            if (gamepad1.left_bumper) {
                intake.intakeBackward();
            }
            // TODO: change to gamepad2
            else if (gamepad1.right_bumper) {
                intake.intakeForward();
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
}
