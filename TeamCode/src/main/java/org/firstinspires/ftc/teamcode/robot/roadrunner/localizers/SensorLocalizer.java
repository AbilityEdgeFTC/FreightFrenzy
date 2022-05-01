package org.firstinspires.ftc.teamcode.robot.roadrunner.localizers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive.MecanumLocalizer;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

@Config
public class SensorLocalizer implements Localizer {
    SampleMecanumDrive mecanumDrive;
    MecanumLocalizer mecanumLocalizer;
    ModernRoboticsI2cColorSensor colorSensor;
    public static double redWhite = 240, blueWhite = 240, greenWhite = 240;
    public static double whiteMax = 255;
    public static double warehouseLineX = 28.5 + 4.72;
    boolean isRed;

    public SensorLocalizer(HardwareMap hardwareMap, boolean isRed)
    {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumLocalizer = new MecanumLocalizer(mecanumDrive, true);
        colorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class, "csG");
        this.isRed = isRed;
    }


    /**
     * Gets the most confident localizer's estimated position
     * @return The position
     */
    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return mecanumDrive.getPoseEstimate();
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        mecanumDrive.setPoseEstimate(pose2d);
    }

    @Override
    public @Nullable Pose2d getPoseVelocity() {
        return mecanumDrive.getPoseVelocity();
    }

    @Override
    public void update() {
        if(compareValueR() && compareValueB() && compareValueG())
        {
            setPoseEstimate(new Pose2d(warehouseLineX, getPoseEstimate().getY(), getPoseEstimate().getHeading()));
        }
    }

    boolean compareValueR()
    {
        return colorSensor.red() >= redWhite && colorSensor.red() <= whiteMax;
    }

    boolean compareValueG()
    {
        return colorSensor.green() >= greenWhite && colorSensor.green() <= whiteMax;
    }

    boolean compareValueB()
    {
        return colorSensor.blue() >= blueWhite && colorSensor.blue() <= whiteMax;
    }



}
