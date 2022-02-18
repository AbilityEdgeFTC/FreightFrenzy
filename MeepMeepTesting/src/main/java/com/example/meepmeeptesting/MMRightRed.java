package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMRightRed {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 12;
        double startPoseRightY = -64.04;
        double startPoseRightH = 0;
        double poseHubFrontX = -11;
        double poseHubFrontY = -41.5;
        double poseHubFrontH = 90;
        double poseEntranceX = 12;
        double poseEntranceY = -64;
        double poseEntranceH = 180;
        double poseCollectX = 60;
        double poseCollectY = -64;
        double poseCollectH = 180;
        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.805)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .lineToLinearHeading(poseHubFront)
                                .waitSeconds(1)
                                .lineToLinearHeading(poseEntrance)
                                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 15, poseCollect.getY(), poseCollect.getHeading() - Math.toRadians(1.5)))
                                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 10, poseCollect.getY(), poseCollect.getHeading() + Math.toRadians(1.5)))
                                .waitSeconds(1)
                                .lineToLinearHeading(poseEntrance)
                                .lineToLinearHeading(poseHubFront)
                                .waitSeconds(1)
                                .lineToLinearHeading(poseEntrance)
                                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 10, poseCollect.getY() - 2, poseCollect.getHeading() - Math.toRadians(2)))
                                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 7.5, poseCollect.getY() - 2, poseCollect.getHeading() + Math.toRadians(3)))
                                .lineToLinearHeading(new Pose2d(poseCollect.getX() - 5, poseCollect.getY() - 2, poseCollect.getHeading() + Math.toRadians(3)))
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