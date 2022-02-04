package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class trajectoryObject {
    private static Pose2d[] points = {};
    private static Pose2d startPose;

    public trajectoryObject(Pose2d[] points,Pose2d startPose) {
        this.points = points;
        this.startPose = startPose;
    }
}
