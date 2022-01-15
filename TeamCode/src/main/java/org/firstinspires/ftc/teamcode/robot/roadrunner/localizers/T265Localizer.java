package org.firstinspires.ftc.teamcode.robot.RoadRunner.localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

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
    public static double LATERAL_DISTANCE = 7.09; // in; offset of the camera from the left or right of the middle of the teleop.
    public static double FORWARD_OFFSET = 0.59; // in; offset of the camera from the front or back of the middle of the teleop.
    public static double DIRECTION = 0;

    private static T265Camera slamra = null;

    public T265Localizer(HardwareMap hardwareMap){
        super();

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

        Pose2d pose = new Pose2d(up.pose.getX() * 39.37, up.pose.getY() * 39.37, up.pose.getHeading() * 180/Math.PI);

        return pose;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX(), pose2d.getY(), Rotation2d.fromDegrees(pose2d.getHeading())));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        ChassisSpeeds up = slamra.getLastReceivedCameraUpdate().velocity;

        Pose2d poseV = new Pose2d(up.vxMetersPerSecond, up.vyMetersPerSecond, up.omegaRadiansPerSecond);

        return poseV;
    }

    @Override
    public void update() {
    }

    /**
     * Gets the last update received from the camera
     * @return The CameraUpdate
     */
    public T265Camera.CameraUpdate getRawUpdate() {
        return slamra.getLastReceivedCameraUpdate();
    }

    /**
     * Sends external odometry to the Realsense camera for extra accuracy
     */
    public void sendOdometry(Pose2d velocity) {
        slamra.sendOdometry(velocity.getX(), velocity.getY());
    }


    /**
     * TODO: ADD THESE MANUALLY TO EVERY END OF OPMODE USING ROADRUNNER!!!!!!!!!!!!!
     */
    public void stop(){
        slamra.stop();
    }

    public void start(HardwareMap hardwareMap){
        //slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        slamra.start();
    }

}