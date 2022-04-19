package org.firstinspires.ftc.teamcode.opmodes.autonmous;

public class MTIConverter {

    double startPoseRightY = -60;
    double startPoseRightH = 90;
    public double poseEntranceX = 9.2;
    public double poseEntranceY = -62.5;
    public double poseEntranceH = 180;
    public double poseCollectX = 59;
    public double poseCollectY = -62;
    public double poseCollectH = 180;
    public double poseHelpX = 7;
    public double poseHelpY = -50;
    double poseHelpH = 180;



    public void convertBlueRightSide(){

        startPoseRightY = -startPoseRightY;
        poseEntranceY = -poseEntranceY;
        poseCollectY = -poseCollectY;
        poseHelpY =  -poseHelpY;

    }


    void BLueLeftSideconvert() {

        poseEntranceX = - poseEntranceX;
        poseCollectX = -poseCollectX;
        poseHelpX = -poseHelpX;
    }


    void convertRedRightSide(){

        startPoseRightY = -startPoseRightY;
        poseEntranceY = -poseEntranceY;
        poseCollectY = -poseCollectY;
        poseHelpY =  -poseHelpY;

    }

}
