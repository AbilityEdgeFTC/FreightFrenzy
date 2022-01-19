package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.Subsystems.ElevatorThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.Subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.Subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.intake;

import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.currentPose;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    DcMotor mC, mFL, mBL, mFR, mBR, mI;
    Servo sD;

    public static double powerCarousel = 0.325;
    public static double powerIntake = 1;
    public static double intakePosition = 1, dippingPosition = .6;
    public static double lockOn = 90;
    public static double mainPower = 1;
    public static boolean isRegularDrive = true;

    carousel carousel;
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        carousel = new carousel(mC, powerCarousel);
        gamepads = new gamepad(gamepad1, gamepad2, mFL, mBL, mFR, mBR, mainPower, isRegularDrive, telemetry, drive, lockOn);

        intake = new intake(mI, powerIntake);
        dip = new dip(sD, intakePosition, dippingPosition);

        Thread ElevatorThread = new ElevatorThread(telemetry, hardwareMap, gamepad2);

        waitForStart();

        ElevatorThread.start();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

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
            }

            // TODO: change to gamepad1
            if(gamepad2.a)
            {
                gamepads.lockOnAngle = !gamepads.lockOnAngle;
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
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                carousel.spin();
            }
            else {
                carousel.stop();
            }
        }

        ElevatorThread.interrupt();
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
        sD = hardwareMap.get(Servo.class, "sE");
    }

}
