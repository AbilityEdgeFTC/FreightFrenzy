/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Static variables are like stored in the memory, that way, we can change the values in different opmodes, and transfer
 * the updated value of them to the other opmodes when presses.
 */
@Config
public class valueStorage
{
    public static Pose2d currentPose = new Pose2d();
    public static double startPoseLeftX = -36;
    public static double startPoseLeftY = -64.24;
    public static double startPoseLeftH = 0;
    public static double startPoseRightX = -12;
    public static double startPoseRightY = -64.24;
    public static double startPoseRightH = 0;
    public static double poseCarousel1X = -50;
    public static double poseCarousel1Y = -36;
    public static double poseCarousel1H = 135;
    public static double poseCarousel2X = -56.7;
    public static double poseCarousel2Y = -64.7;
    public static double poseCarousel2H = 135;
    public static double poseHubFrontX = -9.1;
    public static double poseHubFrontY = -51.5;
    public static double poseHubFrontH = 90;
    public static double poseHubBackX = -9.1;
    public static double poseHubBackY = -51.5;
    public static double poseHubBackH = 90;
    public static double poseHubRightX = -9.1;
    public static double poseHubRightY = -51.5;
    public static double poseHubRightH = 90;
    public static double poseHubLeftX = -9.1;
    public static double poseHubLeftY = -51.5;
    public static double poseHubLeftH = 90;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -70.78;
    public static double poseEntranceH = 180;
    public static double poseEntrance2X = 12;
    public static double poseEntrance2Y = -67;
    public static double poseEntrance2H = 180;
    public static double poseCollectX = 48;
    public static double poseCollectY = -71.8;
    public static double poseCollectH = 180;
    
    public static Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
    public static Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
    public static Pose2d poseCarousel1 = new Pose2d(poseCarousel1X, poseCarousel1Y, Math.toRadians(poseCarousel1H));
    public static Pose2d poseCarousel2 = new Pose2d(poseCarousel2X, poseCarousel2Y, Math.toRadians(poseCarousel2H));
    public static Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseHubBack = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseHubRight = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseHubLeft = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
    public static Pose2d poseEntrance2 = new Pose2d(poseEntrance2X, poseEntrance2Y, Math.toRadians(poseEntrance2H));
    public static Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
}
