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
    public static double poseCarousel1X = -50;
    public static double poseCarousel1Y = -36;
    public static double poseCarousel1H = 135;
    public static double poseCarousel2X = -56.7;
    public static double poseCarousel2Y = -64.7;
    public static double poseCarousel2H = 135;
    public static double poseHubX = -9.1;
    public static double poseHubY = -51.5;
    public static double poseHubH = 90;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -70.78;
    public static double poseEntranceH = 180;
    public static double poseEntrance2X = 12;
    public static double poseEntrance2Y = -67;
    public static double poseEntrance2H = 180;
    public static double poseCollectX = 48;
    public static double poseCollectY = -71.8;
    public static double poseCollectH = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseH));
        Pose2d poseCarousel1 = new Pose2d(poseCarousel1X, poseCarousel1Y, Math.toRadians(poseCarousel1H));
        Pose2d poseCarousel2 = new Pose2d(poseCarousel2X, poseCarousel2Y, Math.toRadians(poseCarousel2H));
        Pose2d poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseEntrance2 = new Pose2d(poseEntrance2X, poseEntrance2Y, Math.toRadians(poseEntrance2H));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        drive.setPoseEstimate(startPose);
        //"Spin Carousel";
        // "Collect And Place Additional Freight", "Number Of Freight To Collect";
        // "Place Freight In";
        // "Park In";
        // "Park Completely";

        Trajectory carousel = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseCarousel1)
                .splineToLinearHeading(poseCarousel2, 0)
                .build();
        Trajectory hub = drive.trajectoryBuilder(carousel.end())
                .lineToSplineHeading(poseHub)
                .build();
        Trajectory collect1 = drive.trajectoryBuilder(hub.end())
                .lineToLinearHeading(poseEntrance)
                .splineToLinearHeading(poseCollect, 0)
                .build();
        Trajectory placement1 = drive.trajectoryBuilder(collect1.end())
                .splineToConstantHeading(new Vector2d(poseEntrance.getX(), poseEntrance.getY()), poseEntrance.getHeading())
                .lineToLinearHeading(poseHub)
                .build();
        /*
        Trajectory collect2 = drive.trajectoryBuilder(placement1.end())
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseCollect, drive.getPoseEstimate().getHeading())
                .build();
        Trajectory placement2 = drive.trajectoryBuilder(collect2.end(), true)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseCollect,  Math.toRadians(0))
                .build();
*/
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
        /*drive.followTrajectory(collect2);
        Thread.sleep(2000);
        drive.followTrajectory(placement2);
*/
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
