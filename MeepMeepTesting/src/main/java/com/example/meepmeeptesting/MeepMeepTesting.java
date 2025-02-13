package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting{

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseLeftX = -35;
        double startPoseLeftY = -72 + 17.72;
        double startPoseLeftH = 90;
        double poseCarouselX = -59.5;
        double poseCarouselY = -57.5;
        double poseCarouselH = 95;
        double poseCaroselInt = 100;



        double carouselHelp = 15;


        double poseParkHelpX = -40;
        double poseParkHelpY = -6;
        double poseParkHelpH = 180;

        double poseParkaX = 7.5;
        double poseParkaY = -6;
        double poseParkaH = 180;

        double poseParkbX = 7.5;
        double poseParkbY = -45;
        double poseParkbH = 180;

        double poseParkcX = 50;
        double poseParkcY = -45;
        double poseParkcH = 180;

        double runCarouselFor = 4;
        //Pose2d turnPoseRight = new Pose2d(turnPoseRightX,turnPoseRidrive
        //SubSystems
        //TeleOp
        //utilghtY,Math.toRadians(turnPoseRightH));
        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));
        Pose2d poseParkinga = new Pose2d(poseParkaX, poseParkaY, Math.toRadians(poseParkaH));
        Pose2d poseParkingb = new Pose2d(poseParkbX, poseParkbY, Math.toRadians(poseParkbH));
        Pose2d poseParkingc = new Pose2d(poseParkcX, poseParkcY, Math.toRadians(poseParkcH));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, 7.706666469573975, 7.706666469573975, 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPoseLeft)
                                .waitSeconds( 1.5)
                                .forward(carouselHelp)
                                .lineToLinearHeading(poseCarousel)
                                .waitSeconds(runCarouselFor)
                                .strafeRight(2)
                                .back(5)
                                .forward(5)
                                .strafeRight(2)
                                .back(5)
                                .forward(5)
                                .strafeRight(2)
                                .back(5)
                                .forward(5)
                                .lineToSplineHeading(poseParkingHelp)
                                .lineToSplineHeading(poseParkinga)
                                .lineToLinearHeading(poseParkingb)
                                .waitSeconds(.6)
                                .lineToLinearHeading(poseParkingc)
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