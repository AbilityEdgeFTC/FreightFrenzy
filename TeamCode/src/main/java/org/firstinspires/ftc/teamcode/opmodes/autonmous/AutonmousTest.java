package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutonmousTest extends LinearOpMode {

    public static double startPoseX = -36;
    public static double startPoseY = -64.24;
    public static double startPoseH = 0;
    public static double poseCarouselX = -58;
    public static double poseCarouselY = -69;
    public static double poseCarouselH = 135;
    public static double poseHubX = -9.5;
    public static double poseHubY = -50;
    public static double poseHubH = 90;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -73.5;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 48;
    public static double poseCollectY = -73.5;
    public static double poseCollectH = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        drive.setPoseEstimate(startPose);
        //"Spin Carousel";
        // "Collect And Place Additional Freight", "Number Of Freight To Collect";
        // "Place Freight In";
        // "Park In";
        // "Park Completely";

        Trajectory carousel = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(poseCarousel.getX(), poseCarousel.getY()), poseCarousel.getHeading())
                .build();
        Trajectory hub = drive.trajectoryBuilder(carousel.end())
                .splineTo(new Vector2d(poseHub.getX(), poseHub.getY()), poseHub.getHeading())
                .build();
        Trajectory collect1 = drive.trajectoryBuilder(hub.end())
                .splineTo(new Vector2d(poseEntrance.getX(), poseEntrance.getY()), poseEntrance.getHeading())
                .splineTo(new Vector2d(poseCollect.getX(), poseCollect.getY()), poseCollect.getHeading())
                .build();
        Trajectory placement1 = drive.trajectoryBuilder(collect1.end())
                .splineTo(new Vector2d(poseHub.getX(), poseHub.getY()), poseHub.getHeading())
                .build();
        Trajectory collect2 = drive.trajectoryBuilder(placement1.end())
                .splineTo(new Vector2d(poseEntrance.getX(), poseEntrance.getY()), poseEntrance.getHeading())
                .splineTo(new Vector2d(poseCollect.getX(), poseCollect.getY()), poseCollect.getHeading())
                .build();
        Trajectory placement2 = drive.trajectoryBuilder(collect2.end())
                .splineTo(new Vector2d(poseHub.getX(), poseHub.getY()), poseHub.getHeading())
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(carousel);
        Thread.sleep(2000);
        drive.followTrajectory(hub);
        Thread.sleep(2000);
        drive.followTrajectory(collect1);
        Thread.sleep(2000);
        drive.followTrajectory(placement1);
        Thread.sleep(2000);
        drive.followTrajectory(collect2);
        Thread.sleep(2000);
        drive.followTrajectory(placement2);

        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
