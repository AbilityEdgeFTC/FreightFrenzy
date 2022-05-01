package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Arrays;
import java.util.Queue;

public class MMRightRedOneOne {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 13;
        double startPoseRightY = -60;
        double startPoseRightH = 90;
        double poseEntranceX = 13;
        double poseEntranceY = -64;
        double poseEntranceH = 180;
        double poseCollectX = 50;
        double poseCollectY = -64;
        double poseCollectH = 180;
        double poseHelpX = 7;
        double poseHelpY = -50;
        double poseHelpH = 180;
        double poseCollectSplineX = 55;
        double poseCollectSplineY = -55;
        double poseCollectSplineH = 200;


        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseCollectSpline = new Pose2d(poseCollectSplineX, poseCollectSplineY, poseCollectSplineH);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 12.74)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .lineToLinearHeading(poseHelp)
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(40, poseCollect.getY(), poseCollect.getHeading()))
                                .splineToSplineHeading(poseCollectSpline, poseCollectSpline.getHeading())
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollectSplineX, poseCollect.getY(), poseCollect.getHeading()))
                                .splineTo(new Vector2d(poseCollectSplineX, poseCollectSplineY), poseCollectSpline.getHeading())
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollectSplineX, poseCollect.getY(), poseCollect.getHeading()))
                                .splineTo(new Vector2d(poseCollectSplineX, poseCollectSplineY), poseCollectSpline.getHeading())
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                                .build()
                );

        myBot.setDimensions(13,17.9);
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}