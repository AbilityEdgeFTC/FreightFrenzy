package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutonmousTest extends LinearOpMode {

    public static double startPoseLeftX = -36;
    public static double startPoseLeftY = -64.24;
    public static double startPoseLeftH = 0;
    public static double poseCarouselX = -60;
    public static double poseCarouselY = -58;
    public static double poseCarouselH = 135;
    public static double carouselHelp = 15;
    public static double poseHubLeftX = -33.1;
    public static double poseHubLeftY = -39.5;
    public static double poseHubLeftH = 0;
    public static double hubHelp = 30;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -67;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 43;
    public static double poseCollectY = -67;
    public static double poseCollectH = 180;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
         Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
         Pose2d poseHubLeft = new Pose2d(poseHubLeftX, poseHubLeftY, Math.toRadians(poseHubLeftH));
         Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
         Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));


        drive.setPoseEstimate(startPoseLeft);
        //"Spin Carousel";
        // "Collect And Place Additional Freight", "Number Of Freight To Collect";
        // "Place Freight In";
        // "Park In";
        // "Park Completely";

        TrajectorySequence auto = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .strafeLeft(carouselHelp)
                .lineToLinearHeading(poseCarousel, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1.5)
                .lineToSplineHeading(poseHubLeft, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1.5)
                .strafeRight(hubHelp)
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(auto);
        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
            valueStorage.currentPose = poseEstimate;
        }


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
