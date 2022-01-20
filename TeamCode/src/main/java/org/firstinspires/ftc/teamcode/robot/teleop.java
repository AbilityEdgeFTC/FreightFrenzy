package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.Subsystems.ElevatorThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.MultitaskingThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.Subsystems.gamepad;

import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.currentPose;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    DcMotor mC, mFL, mBL, mFR, mBR;

    public static double powerCarousel = 0.325;
    public static double lockOn = 90;
    public static double mainPower = 1;
    public static boolean isRegularDrive = true;

    carousel carousel;
    gamepad gamepads;
    SampleMecanumDriveCancelable drive;

    @Override
    public void runOpMode() throws InterruptedException {
        initAll();
        cGamepad cGamepad1 = new cGamepad(gamepad1);
        cGamepad cGamepad2 = new cGamepad(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        carousel = new carousel(mC, powerCarousel);
        gamepads = new gamepad(gamepad1, gamepad2, mFL, mBL, mFR, mBR, mainPower, isRegularDrive, telemetry, drive, lockOn);

        Thread ElevatorThread = new ElevatorThread(telemetry, hardwareMap, gamepad1);
        Thread MultitaskingThread = new MultitaskingThread(telemetry, hardwareMap, gamepad1);

        waitForStart();

        ElevatorThread.start();
        MultitaskingThread.start();

        while (opModeIsActive()) {

            cGamepad1.update();
            cGamepad2.update();
            gamepads.update();

            if(gamepad1.right_stick_button || gamepad1.left_stick_button)
            {
                gamepads.power = .6;
            }
            else
            {
                gamepads.power = 1;
            }

//            // TODO: change to gamepad1
//            if(gamepad2.a)
//            {
//                gamepads.lockOnAngle = !gamepads.lockOnAngle;
//            }

            // TODO: change to gamepad2
            if (gamepad1.dpad_right || gamepad1.dpad_left) {
                carousel.spin();
            }
            else {
                carousel.stop();
            }
        }

        ElevatorThread.interrupt();
        MultitaskingThread.interrupt();
    }

    void initAll() {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mC = hardwareMap.get(DcMotor.class, "mC");
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(currentPose);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
