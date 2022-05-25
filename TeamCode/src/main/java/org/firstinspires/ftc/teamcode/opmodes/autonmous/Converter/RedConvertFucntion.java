package org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter;

import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeCollectX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeCollectY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeEntranceH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeEntranceX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeEntranceY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeHelpH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeHelpX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeHelpY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedposeCollectH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedstartPoseRightX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedstartPoseRightY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.RedPose2DConverter.RedstartPoseRightH;

public class RedConvertFucntion {


    void AutoRedRightSide() {

        RedstartPoseRightX = RedstartPoseRightX * -1;
        RedstartPoseRightY = RedstartPoseRightY * -1;
        RedstartPoseRightH = RedstartPoseRightH * 1;
        RedposeEntranceX = RedposeEntranceX * -1;
        RedposeEntranceY = RedposeEntranceY * -1;
        RedposeEntranceH = RedposeEntranceH * 1;
        RedposeCollectX = RedposeCollectX * -1;
        RedposeCollectY = RedposeCollectY * -1;
        RedposeCollectH = RedposeCollectH * 1;
        RedposeHelpX = RedposeHelpX * -1;
        RedposeHelpY = RedposeCollectY * -1;
        RedposeHelpH = RedposeHelpH * 1;

    }

    void AutoRedLeftSide() {

        RedstartPoseRightX = RedposeEntranceY * -1;
        RedposeEntranceH = RedposeEntranceH * 1;
        RedposeCollectX = RedposeCollectX * 1;
        RedposeCollectY = RedposeCollectY * -1;
        RedposeCollectH = RedposeCollectH * 1;
        RedstartPoseRightX = RedstartPoseRightX * 1;
        RedstartPoseRightY = RedstartPoseRightY * -1;
        RedstartPoseRightH = RedstartPoseRightH * 1;
        RedposeEntranceX = RedposeEntranceX * 1;
        RedposeEntranceY = RedposeHelpX = RedposeHelpX * 1;
        RedposeHelpY = RedposeCollectY * -1;
        RedposeHelpH = RedposeHelpH * 1;

    }
}

