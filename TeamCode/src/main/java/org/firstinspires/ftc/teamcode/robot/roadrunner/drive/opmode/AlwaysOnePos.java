package org.firstinspires.ftc.teamcode.robot.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.DoubleLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.RealsenseLocalizer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
@Config
public class AlwaysOnePos extends LinearOpMode {

    public static double startPoseX = 0;
    public static double startPoseY = 0;
    public static double startPoseH = 0;

    enum RobotLocalizer
    {
        Mecanum,
        Camera,
        Both
    }

    SampleMecanumDrive drive;
    RealsenseLocalizer t265Localizer;

    Pose2d poseEstimate;
    Trajectory trajectory;
    public static RobotLocalizer localizer = RobotLocalizer.Mecanum;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        switch (localizer)
        {
            case Mecanum:
                break;
            case Camera:
                t265Localizer = new RealsenseLocalizer(hardwareMap);
                drive.setLocalizer(t265Localizer);
                break;
            case Both:
                drive.setLocalizer(new DoubleLocalizer(hardwareMap));
                break;
        }

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseH));

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(startPose);

        waitForStart();

        while (!isStopRequested()) {

            try {
                trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(0, 0), 0)
                        .build();

            }catch (EmptyPathSegmentException e)
            {

            }

            if(gamepad1.a)
            {
                try {
                    drive.followTrajectory(trajectory);
                }catch (NullPointerException e)
                {

                }
            }
            poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
}
