package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@Config
@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestT265 extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    public static double LATERAL_DISTANCE = -5.71; // in; offset of the camera from the left or right of the middle of the teleop.
    public static double FORWARD_OFFSET = 2.3062992126; // in; offset of the camera from the front or back of the middle of the teleop.
    public static double DIRECTION = -90;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        if (slamra == null) {
            Translation2d translation2d = new Translation2d(FORWARD_OFFSET / 39.37,LATERAL_DISTANCE / 39.37);
            Rotation2d rotation2d = new Rotation2d(Math.toRadians(DIRECTION));
            slamra = new T265Camera(new Transform2d(translation2d, rotation2d), 0.1, hardwareMap.appContext);
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        final int robotRadius = 10; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Pose X:", translation.getX());
        telemetry.addData("Pose Y:", translation.getY());
        telemetry.addData("Pose HEADING:", rotation.getRadians());
        telemetry.update();
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}