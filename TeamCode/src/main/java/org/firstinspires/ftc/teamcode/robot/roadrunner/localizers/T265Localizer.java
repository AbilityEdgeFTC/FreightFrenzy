package org.firstinspires.ftc.teamcode.robot.roadrunner.localizers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import static org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.T265StartupHook.slamera;

/**
 * Localizer for the T265 camera.
 */
public class T265Localizer implements Localizer {

    T265StartupHook t265StartupHook = new T265StartupHook();

    public T265Localizer(HardwareMap hardwareMap)
    {
        t265StartupHook.createSlamera(hardwareMap.appContext);
    }
    public interface SendOdometryFunction {
        Vector2d run();
    }

    private SendOdometryFunction sendOdometryCallback;

    /**
     * Get the current pose of the T265.
     * @return the pose
     */
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        //return slamera.getLastReceivedCameraUpdate().pose;
        return new Pose2d();
    }

    /**
     * Set the pose of the T265 camera.
     * @param pose2d the pose
     */
    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX(), pose2d.getY(), new Rotation2d(pose2d.getHeading())));
    }

    /**
     * Get the current velocity of the T265.
     * @return the velocity
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return new Pose2d(slamera.getLastReceivedCameraUpdate().velocity.vxMetersPerSecond, slamera.getLastReceivedCameraUpdate().velocity.vyMetersPerSecond, slamera.getLastReceivedCameraUpdate().velocity.omegaRadiansPerSecond);
    }

    /**
     * Send odometry data to the T265 if the callback is set.
     */
    @Override
    public void update() {
        // Make sure the callback is set
        if (sendOdometryCallback != null) {
            Vector2d odometry = sendOdometryCallback.run();
            slamera.sendOdometry(odometry.getX(), odometry.getY());
        }
    }

    /**
     * Set a callback to send odometry data to the T265.
     * @param sendOdometryCallback the callback
     */
    public void setSendOdometryCallback(SendOdometryFunction sendOdometryCallback) {
        this.sendOdometryCallback = sendOdometryCallback;
    }

    public void stop(HardwareMap hardwareMap)
    {
        t265StartupHook.destroySlamera(hardwareMap.appContext);
    }
}