package org.firstinspires.ftc.teamcode.robot.subsystems.menu;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

import java.util.ArrayList;
import java.util.Queue;

public class trajectoryObject {
    Queue<TrajectorySequence> trajectorySequenceQueue = null;
    ArrayList<TrajectorySequence> trajectories;
    MecanumLocalizer drive;
    carousel carousel;
    //ElevatorThreadAuto elevatorThreadAuto;
    dip dip;
    intake intake;
    public static double runCarouselFor = 4;

    public enum TrajectoryType
    {
        CAROUSEL,
        HUB_MIN,
        HUB_MID,
        HUB_MAX,
        INTAKE
    }

    public static TrajectoryType trajectoryType;

    public trajectoryObject(MecanumLocalizer drive, HardwareMap hardwareMap) {
        this.drive = drive;
        //elevatorThreadAuto = new ElevatorThreadAuto(hardwareMap);
        dip = new dip(hardwareMap);
        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
    }

    public void generateTrajectory(Pose2d startPose, Pose2d point, TrajectoryType trajectoryType)
    {
        TrajectorySequence trajectorySequence = null;

        switch (trajectoryType)
        {
            case CAROUSEL:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .UNSTABLE_addTemporalMarkerOffset(0, () -> carousel.spin(true))
                        .UNSTABLE_addTemporalMarkerOffset(runCarouselFor, () -> carousel.stop())
                        .build();
                break;

            case HUB_MIN:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        //.UNSTABLE_addTemporalMarkerOffset(-(elevatorThreadAuto.timeTo+1), () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MIN))
                        //.UNSTABLE_addTemporalMarkerOffset(.5, () -> dip.releaseFreightAUTO())
                        //.UNSTABLE_addTemporalMarkerOffset(1, () -> dip.getFreightAUTO())
                        //.UNSTABLE_addTemporalMarkerOffset(1.5, () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO))
                    .build();
                break;
            case HUB_MID:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        //.UNSTABLE_addTemporalMarkerOffset(-(elevatorThreadAuto.timeTo+1), () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MID))
                        //.UNSTABLE_addTemporalMarkerOffset(.5, () -> dip.releaseFreightAUTO())
                        //.UNSTABLE_addTemporalMarkerOffset(1, () -> dip.getFreightAUTO())
                        //.UNSTABLE_addTemporalMarkerOffset(1.5, () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO))
                        .build();
                break;
            case HUB_MAX:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        //.UNSTABLE_addTemporalMarkerOffset(-(elevatorThreadAuto.timeTo+1), () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MAX))
                        //.UNSTABLE_addTemporalMarkerOffset(.5, () -> dip.releaseFreightAUTO())
                        //.UNSTABLE_addTemporalMarkerOffset(1, () -> dip.getFreightAUTO())
                        //.UNSTABLE_addTemporalMarkerOffset(1.5, () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO))
                        .build();
                break;

            case INTAKE:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point, MecanumLocalizer.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                MecanumLocalizer.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        //.UNSTABLE_addTemporalMarkerOffset(-(trajectorySequence.duration()), () -> intake.intakeForward())
                        //.UNSTABLE_addTemporalMarkerOffset(0, () -> intake.intakeBackward())
                        //.UNSTABLE_addTemporalMarkerOffset(1.5, () -> intake.stop())
                        //.UNSTABLE_addTemporalMarkerOffset(1.5, () -> elevatorThreadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.ZERO))
                        .build();
                break;
        }

        trajectories.add(trajectorySequence);
    }

    public void addTrajectory(TrajectorySequence trajectorySequence)
    {
        trajectorySequenceQueue.add(trajectorySequence);
    }

    public MecanumLocalizer getDrive() {
        return drive;
    }

    public void setDrive(MecanumLocalizer drive) {
        this.drive = drive;
    }

    public TrajectorySequence getTrajectory(int queue)
    {
        return trajectories.get(queue);
    }
    public int getMaxTraj()
    {
        return trajectories.size();
    }
    public void runTrajectories()
    {
        for(int i = 0; i < trajectories.size(); i++)
        {
            trajectorySequenceQueue.add(getTrajectory(i));
        }

        while(!trajectorySequenceQueue.isEmpty()) {
            drive.followTrajectorySequence(trajectorySequenceQueue.poll());
        }
    }
}
