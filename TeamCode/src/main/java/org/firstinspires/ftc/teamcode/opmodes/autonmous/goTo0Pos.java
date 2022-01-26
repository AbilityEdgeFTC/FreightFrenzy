package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.DoubleLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;

@Config
@Autonomous(group = "drive")
public class goTo0Pos extends LinearOpMode {

    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;

    Pose2d desPose;
    DoubleLocalizer doubleLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        doubleLocalizer = new DoubleLocalizer(hardwareMap);
        drive.setLocalizer(doubleLocalizer);

        drive.setPoseEstimate(valueStorage.startPoseLeft);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            desPose = new Pose2d(x,y,heading);
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(desPose)
                    .build();

            drive.followTrajectorySequence(trajSeq);
        }

        doubleLocalizer.stop();
    }
}