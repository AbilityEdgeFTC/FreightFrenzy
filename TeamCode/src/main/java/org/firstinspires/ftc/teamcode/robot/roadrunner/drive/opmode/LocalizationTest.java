package org.firstinspires.ftc.teamcode.robot.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.DoubleLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.T265Localizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
public class LocalizationTest extends LinearOpMode {

    public static double startPoseX = 0;
    public static double startPoseY = 0;
    public static double startPoseH = 0;

    enum RobotLocalizer
    {
        Mecanum,
        Camera,
        Both
    }

    MecanumLocalizer drive;
    Pose2d poseEstimate;
    T265Localizer t265Localizer;

    public static RobotLocalizer localizer = RobotLocalizer.Mecanum;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumLocalizer(hardwareMap);

        switch (localizer)
        {
            case Mecanum:
                break;
            case Camera:
                t265Localizer = new T265Localizer(hardwareMap);
                t265Localizer.start(hardwareMap);
                drive.setLocalizer(t265Localizer);
                break;
            case Both:
                drive.setLocalizer(new DoubleLocalizer(hardwareMap));
                break;
        }

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseH));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(startPose);

        gamepad gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            gamepad.update();

            drive.update();
            poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        if(localizer != RobotLocalizer.Mecanum)
        {
            t265Localizer.stop();
        }
    }
}
