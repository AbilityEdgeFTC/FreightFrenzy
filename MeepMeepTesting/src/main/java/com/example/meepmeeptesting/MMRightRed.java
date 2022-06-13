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

public class MMRightRed {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 13;
        double startPoseRightY = -62;
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

        //Cordinates for each course
        double cylceX2 = 60;
        double cycleY2 = -63;
        double cycleH2 = 15;

        double cylceX3 = 60;
        double cycleY3 = -63;
        double cycleH3 = -45;

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseCollectCycle2 = new Pose2d(cylceX2 , cycleY2, Math.toRadians(cycleH2));
        Pose2d poseCollectCycle3 = new Pose2d(cylceX3 , cycleY3, Math.toRadians(cycleH3));

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(80),
                new RectangleMaskConstraint(53,-72,72,-53,
                        new TranslationalVelocityConstraint(10))));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(80);

//        TrajectorySequence main = new TrajectorySequenceBuilder(startPoseRight, velConstraint, accelConstraint, 7.706666469573975,7.706666469573975)
//                .lineToLinearHeading(poseHelp)
//                .lineToSplineHeading(poseEntrance)
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .waitSeconds(.8)
//                .lineToSplineHeading(poseEntrance)
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .splineTo(new Vector2d(poseCollectCycle2.getX(), poseCollectCycle2.getY()), poseCollectCycle2.getHeading())
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .lineToSplineHeading(poseEntrance)
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseEntrance.getX()+30, poseEntrance.getY(), poseEntrance.getHeading()))
//                .lineToSplineHeading(new Pose2d(poseCollect.getX()+8, poseCollect.getY()+1, poseCollect.getHeading()))
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .lineToSplineHeading(poseEntrance)
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .splineTo(new Vector2d(poseCollectCycle3.getX(), poseCollectCycle3.getY()), poseCollectCycle3.getHeading())
//                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .lineToSplineHeading(poseEntrance)
//                .waitSeconds(.8)
////                                .lineToSplineHeading(new Pose2d(poseEntrance.getX()+30, poseEntrance.getY(), poseEntrance.getHeading()))
////                                .lineToSplineHeading(new Pose2d(poseCollect.getX()+10, poseCollect.getY()-1.5, Math.toRadians(160)))
////                                .waitSeconds(.8)
////                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
////                                .lineToSplineHeading(poseEntrance)
////                                .waitSeconds(.8)
//                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                .build();
        TrajectorySequence main = new TrajectorySequenceBuilder(startPoseRight, velConstraint, accelConstraint, 7.706666469573975,7.706666469573975)
                .lineToLinearHeading(poseHelp)
                .lineToSplineHeading(poseEntrance)
                .waitSeconds(.8)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .splineTo(new Vector2d(poseCollectCycle2.getX(), poseCollectCycle2.getY()), poseCollectCycle2.getHeading())
                .lineToSplineHeading(poseEntrance, new TranslationalVelocityConstraint(80),new ProfileAccelerationConstraint(80))
                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, 7.706666469573975, 7.706666469573975, 13)
                .followTrajectorySequence(main);

        myBot.setDimensions(13,17.9);
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}