package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakeState;

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
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -65;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 48.78;
    public static double poseCollectY = -67;
    public static double poseCollectH = 180;
    intake intake;
    carousel carousel;
    Thread elevatorThread;
    ElevatorThreadAuto elevator;
    MultitaskingThreadAuto multiTask;
    Thread multiTaskThread;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        carousel = new carousel(hardwareMap);
        elevator = new ElevatorThreadAuto(hardwareMap);
        multiTask = new MultitaskingThreadAuto(hardwareMap);
        multiTaskThread = multiTask;
        elevatorThread = elevator;
        intake = new intake(hardwareMap);

        drive.setPoseEstimate(startPoseRight);

        TrajectorySequence placement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseHubFront)
                .build();

        TrajectorySequence entranceFirst = drive.trajectorySequenceBuilder(placement.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect = drive.trajectorySequenceBuilder(entranceFirst.end())
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(34.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(collect.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        elevatorThread.start();
        multiTaskThread.start();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(placement);
        elevator.elevatorState = ElevatorState.MAX;
        elevator.elevatorState = ElevatorState.RELEASE;
        elevator.elevatorState = ElevatorState.ZERO;
        drive.followTrajectorySequence(entranceFirst);
        intake.intakeState = IntakeState.FORWARD;
        drive.followTrajectorySequence(collect);
        intake.intakeState = IntakeState.REVERSE;
        Thread.sleep(1500);
        intake.intakeState = IntakeState.STOP;
        drive.followTrajectorySequence(cycle);
        elevator.elevatorState = ElevatorState.MAX;
        elevator.elevatorState = ElevatorState.RELEASE;
        elevator.elevatorState = ElevatorState.ZERO;
        drive.followTrajectorySequence(entrance);
        intake.intakeState = IntakeState.FORWARD;
        drive.followTrajectorySequence(collect);
        intake.intakeState = IntakeState.REVERSE;
        Thread.sleep(1500);
        intake.intakeState = IntakeState.STOP;
        drive.followTrajectorySequence(cycle);
        elevator.elevatorState = ElevatorState.MAX;
        elevator.elevatorState = ElevatorState.RELEASE;
        elevator.elevatorState = ElevatorState.ZERO;

        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), String.valueOf(drive.getPoseEstimate().getHeading()));
            telemetry.update();
            valueStorage.currentPose = poseEstimate;
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
