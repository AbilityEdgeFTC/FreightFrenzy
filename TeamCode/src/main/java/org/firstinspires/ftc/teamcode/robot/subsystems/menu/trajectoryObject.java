package org.firstinspires.ftc.teamcode.robot.subsystems.menu;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

import java.util.ArrayList;
import java.util.Queue;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL1;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL2;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL3;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.ZERO_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.ZERO_ANGLE;

public class trajectoryObject {
    Queue<TrajectorySequence> trajectorySequenceQueue = null;
    ArrayList<TrajectorySequence> trajectories;
    MecanumLocalizer drive;
    carousel carousel;
    ElevatorAuto elevator;
    ElevatorSpinnerAuto spinner;
    dip dip;
    intake intake;
    hand hand;
    public static double runCarouselFor = 4;

    public enum TrajectoryType
    {
        CAROUSEL,
        HUB_MIN,
        HUB_MID,
        HUB_MAX,
        INTAKE,
        PLAIN
    }

    public static TrajectoryType trajectoryType;

    public trajectoryObject(MecanumLocalizer drive, HardwareMap hardwareMap) {
        this.drive = drive;
        elevator = new ElevatorAuto(hardwareMap);
        spinner = new ElevatorSpinnerAuto(hardwareMap);
        dip = new dip(hardwareMap);
        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
    }

    /**
     * TODO: CHANGE THE TRAJ TYPES FROM lineToLinearHeading TO THE CORRECT TYPE
     */
    public void generateTrajectory(Pose2d startPose, Pose2d point, TrajectoryType trajectoryType, boolean isRed)
    {
        TrajectorySequence trajectorySequence = null;

        switch (trajectoryType)
        {
            case CAROUSEL:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .addTemporalMarker( ( ) -> {
                            if(isRed)
                            {
                                carousel.spin(false);
                            }
                            else
                            {
                                carousel.spin(true);
                            }
                        })
                        .waitSeconds(3.25)
                        .addTemporalMarker( ( ) -> {
                            carousel.stop();
                        })
                        .build();
                break;

            case HUB_MIN:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .addTemporalMarker(pathTime -> pathTime * 0.4, () -> {
                            dip.holdFreight();
                            if(isRed)
                            {
                                spinner.goToPosition(MAX_ANGLE);
                            }
                            else
                            {
                                spinner.goToPosition(MIN_ANGLE);
                            }
                        })
                        .addTemporalMarker(pathTime -> pathTime * 0.6, () -> {
                            elevator.goToPosition(SHARED_HUB);
                        })
                        .addTemporalMarker(pathTime -> pathTime * 0.7, () -> {
                            hand.level1();
                        })
                        .addTemporalMarker(() -> {
                            dip.releaseFreight();
                        })
                        .addTemporalMarker(() -> {
                            dip.getFreight();
                        })
                        .addTemporalMarker(() -> {
                            hand.intake();
                        })
                        .addTemporalMarker(() -> {
                            spinner.goToPosition(ZERO_ANGLE);
                        })
                        .addTemporalMarker(() -> {
                            elevator.goToPosition(ZERO_HEIGHT);
                        })
                    .build();
                break;
            case HUB_MID:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .addTemporalMarker(pathTime -> pathTime * 0.4, () -> {
                            dip.holdFreight();
                            if(isRed)
                            {
                                spinner.goToPosition(MAX_ANGLE);
                            }
                            else
                            {
                                spinner.goToPosition(MIN_ANGLE);
                            }
                        })
                        .addTemporalMarker(pathTime -> pathTime * 0.7, () -> {
                            hand.level1();
                        })
                        .addTemporalMarker(() -> {
                            dip.releaseFreight();
                        })
                        .addTemporalMarker(() -> {
                            dip.getFreight();
                        })
                        .addTemporalMarker(() -> {
                            hand.intake();
                        })
                        .addTemporalMarker(() -> {
                            spinner.goToPosition(ZERO_ANGLE);
                        })
                        .addTemporalMarker(() -> {
                            elevator.goToPosition(ZERO_HEIGHT);
                        })
                        .build();
                break;
            case HUB_MAX:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .addTemporalMarker(pathTime -> pathTime * 0.4, () -> {
                            dip.holdFreight();
                            if(isRed)
                            {
                                spinner.goToPosition(MAX_ANGLE);
                            }
                            else
                            {
                                spinner.goToPosition(MIN_ANGLE);
                            }
                        })
                        .addTemporalMarker(pathTime -> pathTime * 0.7, () -> {
                            hand.level1();
                        })
                        .addTemporalMarker(() -> {
                            dip.releaseFreight();
                        })
                        .addTemporalMarker(() -> {
                            dip.getFreight();
                        })
                        .addTemporalMarker(() -> {
                            hand.intake();
                        })
                        .addTemporalMarker(() -> {
                            spinner.goToPosition(ZERO_ANGLE);
                        })
                        .addTemporalMarker(() -> {
                            elevator.goToPosition(ZERO_HEIGHT);
                        })
                        .build();
                break;

            case INTAKE:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .addTemporalMarker(pathTime -> pathTime * 0.9, () -> {
                            intake.intakeForward();
                        })
                        .waitSeconds(1.5)
                        .addTemporalMarker(() -> {
                            intake.intakeBackward();
                        })
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                            intake.stop();
                        })
                        .build();
                break;
            case PLAIN:
                trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(point)
                        .build();
                break;
        }

        trajectories.add(trajectorySequence);
    }

    public void addTrajectory(TrajectorySequence trajectorySequence)
    {
        trajectories.add(trajectorySequence);
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
