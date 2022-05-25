package org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter;

import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeCollectH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeCollectX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeCollectY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeEntranceH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeEntranceX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeEntranceY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeHelpH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeHelpY;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BlueposeHelpX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BluestartPoseRightH;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BluestartPoseRightX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.Converter.BluePose2DConverter.BluestartPoseRightY;



public class BlueConvertFunction {
    void AutoRedRightSide() {

        BluestartPoseRightX = BluestartPoseRightX * -1;
        BluestartPoseRightY = BluestartPoseRightY * -1;
        BluestartPoseRightH = BluestartPoseRightH * 1;
        BlueposeEntranceX = BlueposeEntranceX * -1;
        BlueposeEntranceY = BlueposeEntranceY * -1;
        BlueposeEntranceH = BlueposeEntranceH * 1;
        BlueposeCollectX = BlueposeCollectX * -1;
        BlueposeCollectY = BlueposeCollectY * -1;
        BlueposeCollectH = BlueposeCollectH * 1;
        BlueposeHelpX = BlueposeHelpX * -1;
        BlueposeHelpY = BlueposeHelpY * -1;
        BlueposeHelpH = BlueposeHelpH * 1;

    }

    void AutoRedLeftSide() {

        BluestartPoseRightX = BlueposeEntranceY * -1;
        BlueposeEntranceH = BlueposeEntranceH * 1;
        BlueposeCollectX = BlueposeCollectX * 1;
        BlueposeCollectY = BlueposeCollectY * -1;
        BlueposeCollectH = BlueposeCollectH * 1;
        BluestartPoseRightX = BluestartPoseRightX * 1;
        BluestartPoseRightY = BluestartPoseRightY * -1;
        BluestartPoseRightH = BluestartPoseRightH * 1;
        BlueposeEntranceX = BlueposeEntranceX * 1;
        BlueposeEntranceY = BlueposeHelpX * 1;
        BlueposeHelpY = BlueposeCollectY * -1;
        BlueposeHelpH = BlueposeHelpH * 1;

    }
}


















