package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class T265Localizer implements Localizer {
    public static double LATERAL_DISTANCE = 0; // in; offset of the camera from the left or right of the middle of the robot.
    public static double FORWARD_OFFSET = 0; // in; offset of the camera from the front or back of the middle of the robot.
    public static double DIRECTION = 90;

    private static T265Camera slamra = null;

    public T265Localizer(HardwareMap hardwareMap){
        Translation2d translation = new Translation2d(LATERAL_DISTANCE,FORWARD_OFFSET);
        Transform2d offset = new Transform2d(translation, Rotation2d.fromDegrees(DIRECTION));

        if (slamra == null) {
            slamra = new T265Camera(offset, 0.1, hardwareMap.appContext);
        }

        setPoseEstimate(new Pose2d(0,0,0));
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        Pose2d pose = new Pose2d(translation.getX(), translation.getY(), rotation.getRadians());

        return pose;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX(), pose2d.getY(), Rotation2d.fromDegrees(pose2d.getHeading())));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;
    }

    /**
     * TODO: ADD THESE MANUALLY TO EVERY END OF OPMODE USING ROADRUNNER!!!!!!!!!!!!!
     */
    public void stop(){
        slamra.stop();
    }

    public void start(){
        slamra.start();
    }

}