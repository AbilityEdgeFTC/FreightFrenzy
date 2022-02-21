package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMRightRedNew {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 12;
        double startPoseRightY = -64.04;
        //double startPoseRightH = 90;
        double startPoseRightH = 180;
        double poseEntranceX = 20;
        double poseEntranceY = -64;
        double poseEntranceH = 180;
        double poseCollectX = 56;
        double poseCollectY = -64;
        double poseCollectH = 180;
        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 12.805)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                //.forward(6)
                                //.lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + .5, poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 1, poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 1.5, poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 2, poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 2.5, poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 3, poseCollect.getY(), poseCollect.getHeading()))
                                .lineToSplineHeading(poseEntrance)
                                .waitSeconds(1.5)
                                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 3.5, poseCollect.getY(), poseCollect.getHeading()))
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