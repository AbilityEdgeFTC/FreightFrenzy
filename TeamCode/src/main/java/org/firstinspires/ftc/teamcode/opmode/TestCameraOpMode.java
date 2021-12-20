package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@Config
@Autonomous(name="Test T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double xCordinate = 0.375;
    public static double yCordinate = -0.375;
    public static double angleCordinate = 0;

    @Override
    public void init() {
        if (slamra == null) {
            //slamra = new T265Camera(new Transform2d(new Translation2d(7.38,10.63),Rotation2d.fromDegrees(0)), 0.1, hardwareMap.appContext);
            slamra = new T265Camera(new Transform2d(new Translation2d(xCordinate,yCordinate),Rotation2d.fromDegrees(angleCordinate)), 0.1, hardwareMap.appContext);

        }

        slamra.setPose(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
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
        final double robotRadius = 6.79; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        telemetry.addData("X: ", translation.getX());
        telemetry.addData("Y: ", translation.getX());
        telemetry.addData("ROTATION: ", up.pose.getRotation());
        telemetry.addData("CONFIDENCE: ", up.confidence);
        telemetry.update();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}