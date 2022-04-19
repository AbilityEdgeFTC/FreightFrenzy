package org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter;

public class Pose2DConverter {

    double startPoseRightX;
    double startPoseRightY;
    double startPoseRightH;
    public double poseEntranceX;
    public double poseEntranceY;
    public double poseEntranceH;
    public double poseCollectX;
    public double poseCollectY;
    public double poseCollectH;
    public double poseHelpX;
    public double poseHelpY;
    double poseHelpH;


    void AutoLeftBlue(){

        startPoseRightX = startPoseRightX * -1;
        startPoseRightY = startPoseRightY * 1;
        startPoseRightH = startPoseRightH * -1;
        poseEntranceX =  poseEntranceX * -1;
        poseEntranceY = poseEntranceY * 1;
        poseEntranceH = poseEntranceH * -1;
        poseCollectX = poseCollectX * -1;
        poseCollectY = poseCollectY * 1;
        poseCollectH = poseCollectH * -1;
        poseHelpX = poseHelpX * -1;
        poseHelpY = poseCollectY * 1;
        poseHelpH = poseHelpH * -1;

    }

    void AutoBlueRightSide(){

        startPoseRightX = startPoseRightX * 1;
        startPoseRightY = startPoseRightY * 1;
        startPoseRightH = startPoseRightH * -1;
        poseEntranceX =  poseEntranceX * 1;
        poseEntranceY = poseEntranceY * 1;
        poseEntranceH = poseEntranceH * -1;
        poseCollectX = poseCollectX * 1;
        poseCollectY = poseCollectY * 1;
        poseCollectH = poseCollectH * -1;
        poseHelpX = poseHelpX * 1;
        poseHelpY = poseCollectY * 1;
        poseHelpH = poseHelpH * -1;


    }

    void AutoRedLeftSide(){

        startPoseRightX = startPoseRightX * -1;
        startPoseRightY = startPoseRightY * -1;
        startPoseRightH = startPoseRightH * 1;
        poseEntranceX =  poseEntranceX * -1;
        poseEntranceY = poseEntranceY * -1;
        poseEntranceH = poseEntranceH * 1;
        poseCollectX = poseCollectX * -1;
        poseCollectY = poseCollectY * -1;
        poseCollectH = poseCollectH * 1;
        poseHelpX = poseHelpX * -1;
        poseHelpY = poseCollectY * -1;
        poseHelpH = poseHelpH * 1;

    }

    void AutoRedRightSide(){

        startPoseRightX = startPoseRightX * 1;
        startPoseRightY = startPoseRightY * -1;
        startPoseRightH = startPoseRightH * 1;
        poseEntranceX =  poseEntranceX * 1;
        poseEntranceY = poseEntranceY * -1;
        poseEntranceH = poseEntranceH * 1;
        poseCollectX = poseCollectX * 1;
        poseCollectY = poseCollectY * -1;
        poseCollectH = poseCollectH * 1;
        poseHelpX = poseHelpX * 1;
        poseHelpY = poseCollectY * -1;
        poseHelpH = poseHelpH * 1;

    }


}
