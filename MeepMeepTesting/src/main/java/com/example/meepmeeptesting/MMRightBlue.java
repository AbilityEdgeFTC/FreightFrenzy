package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MMRightBlue {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 36;
        double startPoseRightY = 55;
        double startPoseRightH = 90;

        double startPoseLeftX = -35;
        double startPoseLeftY = -55;
        double startPoseLeftH = 90;
        double posHighTowerX = -10;
        double poseHighTowerY = -30;
        double poseHighTowerH = 31;

        boolean hasFreight = false, firstTime = true;
        double offset = 0;

        double HighTowerHelp = 15;


        double poseParkHelpX = -40;
        double poseParkHelpY = -6;
        double poseParkHelpH = 180;

        double poseParkX = -65;
        double poseParkY = -35;
        double poseParkH = 0;

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseHighTower = new Pose2d(posHighTowerX, poseHighTowerY, Math.toRadians(poseHighTowerH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX, poseParkHelpY, Math.toRadians(poseParkHelpH));
        Pose2d poseParkingSpotA = new Pose2d(poseParkX, poseParkY, Math.toRadians(poseParkH));
        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, 7.706666469573975, 7.706666469573975, 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseRight)
                                .waitSeconds(1.5)
                                .forward(HighTowerHelp)
                                .lineToLinearHeading(poseHighTower)
                                .waitSeconds(4)
                                .lineToSplineHeading(poseParkingHelp)
                                .lineToSplineHeading(poseParkingSpotA)
                                .waitSeconds(.6)
                                .build()
                );

        myBot.setDimensions(13,17.9);
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}