
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
import org.firstinspires.ftc.teamcode.opmodes.teleop.Carousel;
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

import static org.firstinspires.ftc.teamcode.robot.subsystems.menu.trajectoryObject.TrajectoryType.CAROUSEL;

public class TrajectoryBuilder {
    /**TO DO: TRAJETORY FINAL ARRAY MAYBE**/
    //static trajectoryObject[] trajectorys ={}; /**trajectoryObject ------> PoseEstimate,numOfCycles,park**/
    public static MecanumLocalizer drive = null;
    public static HardwareMap hardwerMap;

    // BLUE:
    public static double startPoseRightBlueX = 9.6;
    public static double startPoseRightBlueY = 64.04;
    public static double startPoseRightBlueH = 180;

    public static double startPoseLeftBlueX = -33.6;
    public static double startPoseLeftBlueY = 64.04;
    public static double startPoseLeftBlueH = 180;


    public static double poseHubFrontBlueX = -11;
    public static double poseHubFrontBlueY = 41.5;
    public static double poseHubFrontBlueH = 270;

    public static double poseHubLeftBlueX = -32.7;
    public static double poseHubLeftBlueY = 21;
    public static double poseHubLeftBlueH = 0;

    public static double poseEntranceBlueX = 12;
    public static double poseEntranceBlueY = 63.5;
    public static double poseEntranceBlueH = 180;

    public static double poseCollectBlueX = 60;
    public static double poseCollectBlueY = 63.5;
    public static double poseCollectBlueH = 180;

    public static double poseCarouselBlueX = -59.5;
    public static double poseCarouselBlueY = 57.5;
    public static double poseCarouselBlueH = 45;
    public static double carouselHelp = 15;

    public static Pose2d startPoseLeftBlue = new Pose2d(startPoseLeftBlueX, startPoseLeftBlueY, Math.toRadians(startPoseLeftBlueH));
    public static Pose2d startPoseRightBlue = new Pose2d(startPoseRightBlueX, startPoseRightBlueY, Math.toRadians(startPoseRightBlueH));
    public static Pose2d PoposeHubFrontBlue = new Pose2d(poseHubFrontBlueX, poseHubFrontBlueY, Math.toRadians(poseHubFrontBlueH));
    public static Pose2d poseEntranceBlue = new Pose2d(poseEntranceBlueX, poseEntranceBlueY, Math.toRadians(poseEntranceBlueH));
    public static Pose2d poseCollectBlue = new Pose2d(poseCollectBlueX, poseCollectBlueY, Math.toRadians(poseCollectBlueH));
    public static Pose2d poseCarouselBlue = new Pose2d(poseCarouselBlueX, poseCarouselBlueY, Math.toRadians(poseCarouselBlueH));
    public static Pose2d poseHubLeftBlue = new Pose2d(poseHubLeftBlueX, poseHubLeftBlueY, Math.toRadians(poseHubLeftBlueH));

    //RED:

    public static double startPoseLeftRedX = -36;
    public static double startPoseLeftRedY = -64.04;
    public static double startPoseLeftRedH = 0;
    public static double startPoseRightX = 12;
    public static double startPoseRightY = -64.04;
    public static double startPoseRightH = 0;

    public static double poseCarouselRedX = -61.5;
    public static double poseCarouselRedY = -57.5;
    public static double poseCarouselRedH = 135;
    public static double carouselHelpRed = 15;
    public static double poseHubLeftRedX = -32;
    public static double poseHubLeftRedY = -21;
    public static double poseHubLeftRedH = 0;
    public static double poseHubFrontRedX = -11;
    public static double poseHubFrontRedY = -41.5;
    public static double poseHubFrontRedH = 90;
    public static double poseEntranceRedX = 12;
    public static double poseEntranceRedY = -64;
    public static double poseEntranceRedH = 180;
    public static double poseCollectRedX = 60;
    public static double poseCollectRedY = -64;
    public static double poseCollectRedH = 180;

    public static Pose2d startPoseLeftRed = new Pose2d(startPoseLeftBlueX, startPoseLeftBlueY, Math.toRadians(startPoseLeftBlueH));
    public static Pose2d startPoseRightRed = new Pose2d(startPoseRightBlueX, startPoseRightBlueY, Math.toRadians(startPoseRightBlueH));
    public static Pose2d PoposeHubFrontRed = new Pose2d(poseHubFrontBlueX, poseHubFrontBlueY, Math.toRadians(poseHubFrontBlueH));
    public static Pose2d poseEntranceRed = new Pose2d(poseEntranceBlueX, poseEntranceBlueY, Math.toRadians(poseEntranceBlueH));
    public static Pose2d poseCollectRed = new Pose2d(poseCollectBlueX, poseCollectBlueY, Math.toRadians(poseCollectBlueH));
    public static Pose2d poseCarouselRed = new Pose2d(poseCarouselBlueX, poseCarouselBlueY, Math.toRadians(poseCarouselBlueH));
    public static Pose2d poseHubLeftRed = new Pose2d(poseHubLeftBlueX, poseHubLeftBlueY, Math.toRadians(poseHubLeftBlueH));

    //____________________________________________________________________________________________________________________________
    static String autonomus;
    static String side;
    static String color;
    static String vision;
    static String delay;
    static String numOfCycles;
    static String park;
    static String carousel;
    static int cycles;

    static boolean defult = false;


    static trajectoryObject HubTrajectory = new trajectoryObject(drive,hardwerMap);
    static trajectoryObject CaruselTrajectory = new trajectoryObject(drive,hardwerMap);
    static trajectoryObject ParkTrajectory = new trajectoryObject(drive,hardwerMap);

    /*
     * make if event for each file you got by the final tasks size.
     */
    public static void tasksRead() {
        /**   **/
        try {
            autonomus = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("RunAutonomous.txt"));
            side = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("AllianceSide.txt"));
            color = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("AllianceColor.txt"));
            vision = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("VisionDetection.txt"));
            delay = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("StartDelay.txt"));
            numOfCycles = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("NumOfCycles.txt"));
            cycles = Integer.parseInt(numOfCycles);
            park = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("ParkIn.txt"));
            carousel = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("Carousel.txt"));
        } catch (NumberFormatException e) {
            defult = false;
        }

    }
    public static void buildTarjectorys()
    {
        //SampleMecanumDriveCancelable drive = null;

       /* if (autonomus == "YES")
        {
            if (color == "BLUE")
            {
                if (side == "LEFT")
                {
                    drive.setPoseEstimate(startPoseLeftBlue);

                    if (numOfCycles != "0")
                    {

                        for (int i = 0; i < cycles; i++)
                        {

                        }
                        if (carousel=="YES")
                        {
                                CaruselTrajectory.generateTrajectory(poseHubLeftBlue,poseCarouselBlue,CAROUSEL,false);
                                //add to list
                                if (park == "Warehouse left")
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                                else if(park == "Warehouse Right")
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                                else if(park == "Warehouse Half")
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                                else
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                            }
                        else
                        {
                                if (park == "Warehouse left")
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                                else if(park == "Warehouse Right")
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                                else if(park == "Warehouse Half")
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                                else
                                {
                                    ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                                }
                            }
                    }
                    else
                    {
                            if (park == "Warehouse left")
                            {
                                ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                            }
                            else if(park == "Warehouse Right")
                            {
                                ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                            }
                            else if(park == "Warehouse Half")
                            {
                                ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                            }
                            else
                            {
                                ParkTrajectory.generateTrajectory(poseCarouselBlue,);
                            }
                        }
                }
                else
                {
                    drive.setPoseEstimate(startPoseRightBlue);
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
                    drive.setPoseEstimate(startPoseLeftRed);
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
                    drive.setPoseEstimate(startPoseRightRed);
                    if (numOfCycles != "0")
                    {
                        if (park == "") {

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
            }
        }
        else
        {
            defult = true;
        }

        */
    }

    //start the trajectorys? or save them to file again and the to use the
}


