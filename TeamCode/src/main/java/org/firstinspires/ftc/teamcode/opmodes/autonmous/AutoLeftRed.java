package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake.IntakeState;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;

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
    public static double runCarouselFor = 4;
    public static double helpPark = 5;
    carousel carousel;
    Elevator elevator;
    intake intake;
    dip dip;
    public static double timeTo = 1;
    public static double reverseIntakeFor = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseHubLeft = new Pose2d(poseHubLeftX, poseHubLeftY, Math.toRadians(poseHubLeftH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        carousel = new carousel(hardwareMap);
        elevator = new Elevator(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);

        drive.setPoseEstimate(startPoseLeft);

        TrajectorySequence carouselGo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(carouselHelp)
                .lineToLinearHeading(poseCarousel)
                .build();

        TrajectorySequence hub = drive.trajectorySequenceBuilder(carouselGo.end())
                .lineToSplineHeading(poseHubLeft)
                .build();

        TrajectorySequence entrance = drive.trajectorySequenceBuilder(hub.end())
                .strafeRight(hubHelp)
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect = drive.trajectorySequenceBuilder(entrance.end())
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(collect.end())
                .strafeTo(new Vector2d(poseCollect.getX() + helpPark, poseCollect.getY() + helpPark))
                .strafeLeft(helpPark)
                .build();

        NanoClock clock = NanoClock.system();

        waitForStart();

        double startTime = clock.seconds();

        drive.followTrajectorySequence(carouselGo);
        runCarousel();
        drive.followTrajectorySequence(hub);
        goToMax(clock, startTime);
        drive.followTrajectorySequence(entrance);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        fixIntake();
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

    }

    void goToMax(NanoClock clock, double startTime) throws InterruptedException {
        elevator.setHeight(Elevator.MAX_HEIGHT);
        while (isStopRequested() && (clock.seconds() - startTime) < timeTo)
        {
            elevator.update();
        }
        dip.releaseFreightPos();
        dip.releaseFreight();
        elevator.setHeight(Elevator.ZERO_HEIGHT);
        dip.getFreight();
        while (isStopRequested() && (clock.seconds() - startTime) < timeTo)
        {
            elevator.update();
        }
    }

    void fixIntake() throws InterruptedException {
        intake.intakeBackward();
        Thread.sleep((long)(reverseIntakeFor * 1000));
        intake.stop();
    }

    void runCarousel() throws InterruptedException {
        carousel.spin(true, false);
        Thread.sleep((long)(runCarouselFor * 1000));
        carousel.stop();
    }
}
