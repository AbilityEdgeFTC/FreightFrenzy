package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.LineSegment;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.acmerobotics.roadrunner.path.heading.WiggleInterpolator;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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

public class MMRightRed {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double startPoseRightX = 13;
        double startPoseRightY = -60;
        double startPoseRightH = 90;
        double poseEntranceX = 13;
        double poseEntranceY = -64;
        double poseEntranceH = 180;
        double poseCollectX = 60;
        double poseCollectY = -64;
        double poseCollectH = 180;
        double poseHelpX = 7;
        double poseHelpY = -53;
        double poseHelpH = 180;

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        MaskTest rectangleMask = new MaskTest(55,-72,72,-55,
                new TranslationalVelocityConstraint(20));

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(80),
                rectangleMask));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(50);

        // the path
        LineSegment line1 = new LineSegment(
                new Vector2d(poseEntranceX, poseEntranceY),
                new Vector2d(poseCollectX, poseCollectY)
        );

        // the path
        LineSegment line2 = new LineSegment(
                new Vector2d(40, poseEntranceY),
                new Vector2d(poseCollectX, poseCollectY)
        );

        // wiggle controller
        WiggleInterpolator interp = new WiggleInterpolator(0.13,14, new TangentInterpolator(3/2 * Math.PI));

        // path without wiggle
        PathSegment segmentWithout = new PathSegment(line1, new ConstantInterpolator(Math.PI));

        // path with wiggle
        PathSegment segmentWith = new PathSegment(line2, interp);

        // controls velocity through out the path
        Trajectory trajWithout = new TrajectoryBuilder(poseEntrance, velConstraint, accelConstraint)
                .lineTo(new Vector2d(40, poseCollectY))
                .build();

        // controls velocity through out the path
        Trajectory trajWith = new TrajectoryBuilder(new Pose2d(40, poseEntranceY, poseEntranceH), velConstraint, accelConstraint)
                .lineTo(new Vector2d(60, poseCollectY))
                .build();

        // controls velocity through out the path
        Trajectory trajFix = new TrajectoryBuilder(new Pose2d(60, poseCollectY, poseCollectH), velConstraint, accelConstraint)
                .lineToLinearHeading(poseEntrance)
                .build();

        // profile of traj
        MotionProfile profileWithout = trajWith.getProfile();
        MotionProfile profileWith = trajWithout.getProfile();

        // path with the wiggle
        Path pathWithout = new Path(segmentWithout);
        Path pathWith = new Path(segmentWith);

        Trajectory trajectoryWithout = new Trajectory(pathWithout, profileWithout);
        Trajectory trajectoryWith = new Trajectory(pathWith, profileWith);

        TrajectorySequence trajectorySequence = new TrajectorySequenceBuilder(poseEntrance, velConstraint, accelConstraint, 7.314444541931152,7.314444541931152)
                .addTrajectory(trajectoryWithout)
                .addTrajectory(trajectoryWith)
                .resetConstraints()
                .addTrajectory(trajFix)
                .waitSeconds(1.5)
                .lineToSplineHeading(poseCollect)
                .lineToSplineHeading(poseEntrance)
                .waitSeconds(1.5)
                .lineToSplineHeading(poseCollect)
                .lineToSplineHeading(poseEntrance)
                .waitSeconds(1.5)
                .lineToSplineHeading(poseCollect)
                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, 7.314444541931152, 7.314444541931152, 12.74)
                .followTrajectorySequence(trajectorySequence);

//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(80, 80, Math.toRadians(270), Math.toRadians(270), 12.74)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(startPoseRight)
//                                .lineToLinearHeading(poseHelp)
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(1.5)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(1.5)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(1.5)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(1.5)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(1.5)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(1.5)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                        .build());

        myBot.setDimensions(13,17.9);
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}