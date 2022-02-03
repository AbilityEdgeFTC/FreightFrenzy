package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.ServoThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;


/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoRightRed extends LinearOpMode {

    public static double startPoseRightX = 12;
    public static double startPoseRightY = -64.24;
    public static double startPoseRightH = 0;
    public static double poseHubFrontX = -11.7;
    public static double poseHubFrontY = -45;
    public static double poseHubFrontH = 90;
    public static double poseHubRightX = .3;
    public static double poseHubRightY = -45;
    public static double poseHubRightH = 135;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -67;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 48.78;
    public static double poseCollectY = -67;
    public static double poseCollectH = 180;
    public static int numOfCycles = 5;
    carousel carousel;
    Thread elevatorThread;
    ElevatorThreadAuto elevator;
    ServoThreadAuto multiTask;
    Thread multiTaskThread;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
        Pose2d poseHubRight = new Pose2d(poseHubRightX, poseHubRightY, Math.toRadians(poseHubRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        carousel = new carousel(hardwareMap);
        elevator = new ElevatorThreadAuto(hardwareMap);
        multiTask = new ServoThreadAuto(hardwareMap);
        multiTaskThread = multiTask;
        elevatorThread = elevator;

        drive.setPoseEstimate(startPoseRight);

        Trajectory placement = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseHubFront, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.5, 0, () -> {
                    elevator.elevatorState = ElevatorState.MAX;
                })
                .build();


        TrajectorySequence collectFirst = drive.trajectorySequenceBuilder(placement.end())
                .addTemporalMarker(0.1, 0, () -> {
                    elevator.elevatorState = ElevatorState.ZERO;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    multiTask.intakeState = ServoThreadAuto.IntakeState.FORWARD;
                })
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(collectFirst.end())
                .addTemporalMarker(0.1, 0, () -> {
                    elevator.elevatorState = ElevatorState.ZERO;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    multiTask.intakeState = ServoThreadAuto.IntakeState.FORWARD;
                })
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(34.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, 0, () -> {
                    multiTask.intakeState = ServoThreadAuto.IntakeState.REVERSE;
                })
                .addTemporalMarker(0.1, 3, () -> {
                    multiTask.intakeState = ServoThreadAuto.IntakeState.STOP;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.8, 0, () -> {
                    elevator.elevatorState = ElevatorState.MAX;
                })
                .lineToLinearHeading(poseHubRight, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        elevatorThread.start();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(placement);
        elevator.elevatorState = ElevatorState.RELEASE;
        drive.followTrajectorySequence(collectFirst);
        drive.followTrajectorySequence(cycle);
        elevator.elevatorState = ElevatorState.RELEASE;
        drive.followTrajectorySequence(cycle);
        elevator.elevatorState = ElevatorState.RELEASE;
        drive.followTrajectorySequence(cycle);
        elevator.elevatorState = ElevatorState.RELEASE;

        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), String.valueOf(drive.getPoseEstimate().getHeading()));
            telemetry.update();
            valueStorage.currentPose = poseEstimate;
            elevatorThread.interrupt();
        }

        elevatorThread.interrupt();


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
