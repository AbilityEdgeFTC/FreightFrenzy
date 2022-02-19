package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        double poseEntranceX = 10;
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
                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 12.805)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .lineToLinearHeading(poseHubFront)
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
                                .lineToSplineHeading(poseEntrance)
                                .splineTo(new Vector2d(poseHubFront.getX(), poseHubFront.getY()), poseHubFront.getHeading())
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
                                .lineToSplineHeading(poseEntrance)
                                .splineTo(new Vector2d(poseHubFront.getX(), poseHubFront.getY()), poseHubFront.getHeading())
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
                                .lineToSplineHeading(poseEntrance)
                                .splineTo(new Vector2d(poseHubFront.getX(), poseHubFront.getY()), poseHubFront.getHeading())
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
                                .lineToSplineHeading(poseEntrance)
                                .splineTo(new Vector2d(poseHubFront.getX(), poseHubFront.getY()), poseHubFront.getHeading())
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
                                .lineToSplineHeading(poseEntrance)
                                .splineTo(new Vector2d(poseHubFront.getX(), poseHubFront.getY()), poseHubFront.getHeading())
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
                                .lineToSplineHeading(poseEntrance)
                                .splineTo(new Vector2d(poseHubFront.getX(), poseHubFront.getY()), poseHubFront.getHeading())
                                .back(5)
                                .splineToSplineHeading(poseEntrance, Math.toRadians(0))
                                .lineToSplineHeading (poseCollect)
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