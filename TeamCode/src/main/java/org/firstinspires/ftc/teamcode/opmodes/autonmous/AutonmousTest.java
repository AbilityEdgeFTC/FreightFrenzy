package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutonmousTest extends LinearOpMode {

    public static Pose2d currentPose = new Pose2d();
    public static double startPoseLeftX = -36;
    public static double startPoseLeftY = -64.24;
    public static double startPoseLeftH = 0;
    public static double startPoseRightX = -12;
    public static double startPoseRightY = -64.24;
    public static double startPoseRightH = 0;
    /*public static double poseCarousel1X = -50;
    public static double poseCarousel1Y = -36;
    public static double poseCarousel1H = 135;
     */
    public static double poseCarousel2X = -64;
    public static double poseCarousel2Y = -66;
    public static double poseCarousel2H = 135;
    public static double poseHubFrontX = -9.1;
    public static double poseHubFrontY = -51.5;
    public static double poseHubFrontH = 90;
    public static double poseHubBackX = -9.1;
    public static double poseHubBackY = -51.5;
    public static double poseHubBackH = 90;
    public static double poseHubRightX = -9.1;
    public static double poseHubRightY = -51.5;
    public static double poseHubRightH = 90;
    public static double poseHubLeftX = -9.1;
    public static double poseHubLeftY = -51.5;
    public static double poseHubLeftH = 90;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -71.3;
    public static double poseEntranceH = 180;
    /*
    public static double poseEntrance2X = 12;
    public static double poseEntrance2Y = -67;
    public static double poseEntrance2H = 180;
     */
    public static double poseCollectX = 55;
    public static double poseCollectY = -71.3;
    public static double poseCollectH = 180;

    public static Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
    public static Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));

    public static Pose2d poseCarousel = new Pose2d(poseCarousel2X, poseCarousel2Y, Math.toRadians(poseCarousel2H));
    public static Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseHubBack = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseHubRight = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseHubLeft = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
    // public static Pose2d poseEntrance2 = new Pose2d(poseEntrance2X, poseEntrance2Y, Math.toRadians(poseEntrance2H));
    public static Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //valueStorage valueStorage = new valueStorage();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPoseLeft);
        //"Spin Carousel";
        // "Collect And Place Additional Freight", "Number Of Freight To Collect";
        // "Place Freight In";
        // "Park In";
        // "Park Completely";

        TrajectorySequence auto = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseCarousel)
                .waitSeconds(1.5)
                .lineToSplineHeading(poseHubFront)
                .waitSeconds(1.5)
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseCollect)
                //.splineToLinearHeading(poseCollect, 0)
                .waitSeconds(1.5)
                .lineToLinearHeading(poseEntrance)
                //.splineToConstantHeading(new Vector2d(poseEntrance.getX(), poseEntrance.getY()), poseEntrance.getHeading())
                .lineToLinearHeading(poseHubFront)//more left strange
                .waitSeconds(1.5)
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


        drive.followTrajectorySequence(auto);
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
