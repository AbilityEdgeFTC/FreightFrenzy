package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
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
@Autonomous(name = "Right Red FULL", group = "red")
public class AutoRightRed extends LinearOpMode {

    public static double startPoseRightX = 12;
    public static double startPoseRightY = -64.24;
    public static double startPoseRightH = 0;
    public static double poseHubFrontX = -12;
    public static double poseHubFrontY = -42;
    public static double poseHubFrontH = 90;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -64;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 60;
    public static double poseCollectY = -64;
    public static double entranceHelp = 2;
    public static double poseCollectH = 180;
    carousel carousel;
    intake intake;
    dip dip;
    ElevatorThreadAuto threadAuto;
    public static double reverseIntakeFor = .8;

    //TrajectorySequence collect, placement, collect2, collect3, entrance, entrance2, entrance3, cycle, cycle2;
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

        //, SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
        //                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

        TrajectorySequence placement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance = drive.trajectorySequenceBuilder(placement.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect = drive.trajectorySequenceBuilder(entrance.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-15,poseCollect.getY(),poseCollect.getHeading() - Math.toRadians(1.5)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-10,poseCollect.getY(),poseCollect.getHeading() + Math.toRadians(1.5)))
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(collect.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance2 = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect2 = drive.trajectorySequenceBuilder(entrance2.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-10,poseCollect.getY()-2,poseCollect.getHeading() - Math.toRadians(2)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-5,poseCollect.getY()-2,poseCollect.getHeading() + Math.toRadians(2)))
                .build();

        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(collect2.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance3 = drive.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(poseEntrance)
                .strafeLeft(entranceHelp)
                .build();

        TrajectorySequence collect3 = drive.trajectorySequenceBuilder(entrance3.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-5,poseCollect.getY(),poseCollect.getHeading() - Math.toRadians(2.5)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-2,poseCollect.getY(),poseCollect.getHeading() + Math.toRadians(2.5)))
                .build();


        threadAuto.start();
        dip.getFreight();

        waitForStart();

        if (isStopRequested())  threadAuto.interrupt();


        drive.followTrajectorySequence(placement);
        goToMax();
        drive.followTrajectorySequence(entrance);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        Thread.sleep(1500);
        fixIntake();
        drive.followTrajectorySequence(cycle);
        goToMax();
        drive.followTrajectorySequence(entrance2);
        intake.intakeForward();
        drive.followTrajectorySequence(collect2);
        Thread.sleep(1500);
        fixIntake();
        drive.followTrajectorySequence(cycle2);
        goToMax();
        drive.followTrajectorySequence(entrance3);
        intake.intakeForward();
        drive.followTrajectorySequence(collect3);
        Thread.sleep(1500);
        fixIntake();
        threadAuto.interrupt();

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
        Thread.sleep(800);
        dip.releaseFreightPos();
        dip.releaseFreight();
        dip.getFreight();
        threadAuto.setElevatorState(ElevatorState.ZERO);
    }

    void fixIntake() throws InterruptedException {
        intake.intakeBackward();
        Thread.sleep((long)(reverseIntakeFor * 1000));
        intake.stop();
    }

}
