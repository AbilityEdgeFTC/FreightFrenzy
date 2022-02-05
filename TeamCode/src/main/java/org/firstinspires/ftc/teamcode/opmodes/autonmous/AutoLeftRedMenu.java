package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robot.trajectoryObject;

@Config
@Autonomous(group = "drive")
public class AutoLeftRedMenu extends LinearOpMode {

    public static double startPoseLeftX = -36;
    public static double startPoseLeftY = -64.24;
    public static double startPoseLeftH = 0;
    public static double poseCarouselX = -60.5;
    public static double poseCarouselY = -58.5;
    public static double poseCarouselH = 135;
    public static double carouselHelp = 15;
    public static double poseHubLeftX = -33.1;
    public static double poseHubLeftY = -20;
    public static double poseHubLeftH = 0;
    public static double hubHelp = 15;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -67;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 43;
    public static double poseCollectY = -67;
    public static double poseCollectH = 180;
    public static double helpPark = 5;
    carousel carousel;
    Thread elevatorThread;
    ElevatorThreadAuto elevator;
    MultitaskingThreadAuto multiTask;
    Thread multiTaskThread;
    intake intake;
    trajectoryObject trajectories;

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

        carousel = new carousel(hardwareMap);
        elevator = new ElevatorThreadAuto(hardwareMap);
        multiTask = new MultitaskingThreadAuto(hardwareMap);
        multiTaskThread = multiTask;
        elevatorThread = elevator;
        intake = new intake(hardwareMap);
        trajectories = new trajectoryObject(drive, hardwareMap);

        TrajectorySequence carouselHelper = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(carouselHelp)
                .lineToLinearHeading(poseCarousel)
                .build();
        trajectories.addTrajectory(carouselHelper);
        trajectories.generateTrajectory(drive.getPoseEstimate(), poseCarousel, trajectoryObject.TrajectoryType.CAROUSEL);
        trajectories.generateTrajectory(trajectories.getTrajectory(trajectories.getMaxTraj()-1).end(), poseHubLeft, trajectoryObject.TrajectoryType.HUB_MAX);
        TrajectorySequence hubHelper = drive.trajectorySequenceBuilder(trajectories.getTrajectory(trajectories.getMaxTraj()-1).end())
                .strafeRight(hubHelp)
                .build();
        trajectories.addTrajectory(hubHelper);
        TrajectorySequence entrance = drive.trajectorySequenceBuilder(trajectories.getTrajectory(trajectories.getMaxTraj()-1).end())
                .lineToLinearHeading(poseEntrance)
                .build();
        trajectories.addTrajectory(entrance);
        trajectories.generateTrajectory(trajectories.getTrajectory(trajectories.getMaxTraj()-1).end(), poseCollect, trajectoryObject.TrajectoryType.INTAKE);
        TrajectorySequence park = drive.trajectorySequenceBuilder(trajectories.getTrajectory(trajectories.getMaxTraj()-1).end())
                .strafeTo(new Vector2d(poseCollect.getX() + helpPark, poseCollect.getY() + helpPark))
                .strafeLeft(helpPark)
                .build();
        trajectories.addTrajectory(park);

        elevatorThread.start();
        multiTaskThread.start();

        if (isStopRequested())
        {
            elevatorThread.interrupt();
            multiTaskThread.interrupt();
        }

        waitForStart();

        if (isStopRequested())
        {
            elevatorThread.interrupt();
            multiTaskThread.interrupt();
        }

        trajectories.runTrajectories();

        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
            valueStorage.currentPose = poseEstimate;
            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), String.valueOf(drive.getPoseEstimate().getHeading()));
        }

        elevatorThread.interrupt();
        multiTaskThread.interrupt();

        if(isStopRequested())
        {
            elevatorThread.interrupt();
            multiTaskThread.interrupt();
        }

    }
}
