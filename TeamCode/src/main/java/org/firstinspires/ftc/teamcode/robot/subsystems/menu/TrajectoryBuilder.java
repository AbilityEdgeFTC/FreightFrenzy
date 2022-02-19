
package org.firstinspires.ftc.teamcode.robot.subsystems.menu;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.GreenLanternPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
public class TrajectoryBuilder {
    /**TO DO: TRAJETORY FINAL ARRAY MAYBE**/
    //static trajectoryObject[] trajectorys ={}; /**trajectoryObject ------> PoseEstimate,numOfCycles,park**/
    public static MecanumLocalizer drive = null;
    public static HardwareMap hardwerMap;

    // BLUE:
    public static double startPoseRightX = 9.6;
    public static double startPoseRightY = 64.04;
    public static double startPoseRightH = 180;

    public static double startPoseLeftX = -33.6;
    public static double startPoseLeftY = 64.04;
    public static double startPoseLeftH = 180;


    public static double poseHubFrontX = -11;
    public static double poseHubFrontY = 41.5;
    public static double poseHubFrontH = 270;

    public static double poseHubLeftX = -32.7;
    public static double poseHubLeftY = 21;
    public static double poseHubLeftH = 0;

    public static double poseEntranceX = 12;
    public static double poseEntranceY = 63.5;
    public static double poseEntranceH = 180;

    public static double poseCollectX = 60;
    public static double poseCollectY = 63.5;
    public static double poseCollectH = 180;

    public static double poseCarouselX = -59.5;
    public static double poseCarouselY = 57.5;
    public static double poseCarouselH = 45;
    public static double carouselHelp = 15;

    public static Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
    public static Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
    public static Pose2d PoposeHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
    public static Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
    public static Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
    public static Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
    public static Pose2d poseHubLeft = new Pose2d(poseHubLeftX, poseHubLeftY, Math.toRadians(poseHubLeftH));

    //RED:




    static String autonomus;
    static String side;
    static String color;
    static String vision;
    static String delay;
    static String numOfCycles;
    static String park;
    static boolean defult = false;


    static trajectoryObject hub = new trajectoryObject(drive,hardwerMap);

    /*
     * make if event for each file you got by the final tasks size.
     */
    public static void tasksRead() {
        /**   **/
        try {
            autonomus = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("autonomus.txt"));
            side = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("side.txt"));
            color = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("color.txt"));
            vision = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("color.txt"));
            delay = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("vision.txt"));
            numOfCycles = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("numOfCycles.txt"));
            park = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("park.txt"));
        } catch (NumberFormatException e) {
            defult = false;
        }

    }
    public static void buildTarjectorys()
    {
        //SampleMecanumDriveCancelable drive = null;

        if (autonomus == "YES")
        {
            if (color == "BLUE")
            {
                if (side == "LEFT")
                {
                    drive.setPoseEstimate(startPoseLeft);

                    if (numOfCycles != "0")
                    {

                        if (park == "")
                        {

                        }
                        else
                        {

                        }
                    }
                    else
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                }
                else
                {
                    drive.setPoseEstimate(startPoseRight);
                    if (numOfCycles != "0")
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                    else
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                }
            }
            else
            {
                if (side == "LEFT")
                {
                    if (numOfCycles != "0")
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                    else
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                }
                else
                {
                    if (numOfCycles != "0")
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                    else
                    {
                        if (park == "") {

                        } else {

                        }
                    }
                }
            }
        }
        else
        {
            defult = true;
        }
    }

    //start the trajectorys? or save them to file again and the to use the
}


