package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoLeftBlue extends LinearOpMode {

    public static double startPoseRightX = 12;
    public static double startPoseRightY = 64.24;
    public static double startPoseRightH = 180;
    public static double poseHubFrontX = -11.7;
    public static double poseHubFrontY = 45;
    public static double poseHubFrontH = 270;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = 65;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 50;
    public static double poseCollectY = 67;
    public static double poseCollectH = 180;
    carousel carousel;
    intake intake;
    dip dip;
    ElevatorThreadAuto threadAuto;
    public static double reverseIntakeFor = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        threadAuto = new ElevatorThreadAuto(hardwareMap);

        drive.setPoseEstimate(startPoseRight);

        TrajectorySequence placement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseHubFront)
                .build();

        TrajectorySequence entranceFirst = drive.trajectorySequenceBuilder(placement.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect = drive.trajectorySequenceBuilder(entranceFirst.end())
                .lineToSplineHeading(new Pose2d(poseCollect.getX()-20,poseCollect.getY(),poseCollect.getHeading()))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()-15,poseCollect.getY(),poseCollect.getHeading()))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()-10,poseCollect.getY(),poseCollect.getHeading() - Math.PI / 8), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()-5,poseCollect.getY(),poseCollect.getHeading() + Math.PI / 8), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()-2,poseCollect.getY(),poseCollect.getHeading() - Math.PI / 8), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(poseCollect.getX(),poseCollect.getY(),poseCollect.getHeading() + Math.PI / 8), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(collect.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        threadAuto.start();
        dip.getFreight();

        waitForStart();

        if (isStopRequested())  threadAuto.interrupt();

        drive.followTrajectorySequence(placement);
        goToMax();
        drive.followTrajectorySequence(entranceFirst);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        fixIntake();
        drive.followTrajectorySequence(cycle);
        goToMax();
        drive.followTrajectorySequence(entrance);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        fixIntake();
        drive.followTrajectorySequence(cycle);
        goToMax();
        drive.followTrajectorySequence(entrance);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        fixIntake();

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

        threadAuto.interrupt();

    }

    void goToMax() throws InterruptedException {
        threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MAX);
        Thread.sleep(1000);
        dip.releaseFreightPos();
        dip.releaseFreight();
        threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MIN);
        Thread.sleep(3000);
        dip.getFreight();
    }

    void fixIntake() throws InterruptedException {
        intake.intakeBackward();
        Thread.sleep((long)(reverseIntakeFor * 1000));
        intake.stop();
    }
}
