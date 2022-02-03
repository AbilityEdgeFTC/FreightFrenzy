package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.ServoThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;

import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
@Autonomous(group = "drive")
public class AutoLeftRed extends LinearOpMode {

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
    public static double poseParkX = 35;
    public static double poseParkY = -43;
    public static double poseParkH = 135;
    public static double runCarouselFor = 4;
    carousel carousel;
    Thread elevatorThread;
    ElevatorThreadAuto elevator;
    ServoThreadAuto multiTask;
    Thread multiTaskThread;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
         Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
         Pose2d poseHubLeft = new Pose2d(poseHubLeftX, poseHubLeftY, Math.toRadians(poseHubLeftH));
         Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
         Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
         Pose2d posePark = new Pose2d(poseParkX, poseParkY, Math.toRadians(poseParkH));
         carousel = new carousel(hardwareMap);
        elevator = new ElevatorThreadAuto(hardwareMap);
        multiTask = new ServoThreadAuto(hardwareMap);
        multiTaskThread = multiTask;
        elevatorThread = elevator;

        drive.setPoseEstimate(startPoseLeft);

        TrajectorySequence carouselGo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(carouselHelp)
                .lineToLinearHeading(poseCarousel, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    carousel.spin(true, false);
                })
                .waitSeconds(runCarouselFor)
                .build();

        TrajectorySequence hub = drive.trajectorySequenceBuilder(carouselGo.end())
                .lineToSplineHeading(poseHubLeft, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, 0, () -> {
                    elevator.elevatorState = ElevatorState.MAX;
                })
                .build();

        TrajectorySequence auto = drive.trajectorySequenceBuilder(hub.end())
                .addTemporalMarker(0.1, 0, () -> {
                    elevator.elevatorState = ElevatorState.ZERO;
                })
                .strafeRight(hubHelp, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    multiTask.intakeState = ServoThreadAuto.IntakeState.FORWARD;
                })
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(2)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(auto.end())
                .lineToLinearHeading(posePark)
                .build();

        elevatorThread.start();
        multiTaskThread.start();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(carouselGo);
        carousel.stop();
        drive.followTrajectorySequence(hub);
        elevator.elevatorState = ElevatorState.RELEASE;
        drive.followTrajectorySequence(auto);
        multiTask.intakeState = ServoThreadAuto.IntakeState.REVERSE;
        Thread.sleep(1500);
        multiTask.intakeState = ServoThreadAuto.IntakeState.STOP;
        drive.followTrajectorySequence(park);

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

        if(isStopRequested())
        {
            elevatorThread.interrupt();
            multiTaskThread.interrupt();
        }

    }
}
