package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.jetbrains.annotations.NotNull;

public class RectangleMaskConstraint implements TrajectoryVelocityConstraint {
    double minX, minY, maxX, maxY;
    TrajectoryVelocityConstraint maxVelocity;

    public RectangleMaskConstraint(double minX, double minY, double maxX, double maxY, TrajectoryVelocityConstraint maxVelocity) {
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
        this.maxVelocity = maxVelocity;
    }

    /**
     * Returns the maximum profile velocity.
     *
     * @param s path displacement
     * @param pose path pose
     * @param deriv pose derivative
     * @param baseRobotVel additive base velocity
     */
    @Override
    public double get(double s, @NotNull Pose2d pose, @NotNull Pose2d deriv, @NotNull Pose2d baseRobotVel) {
        if(minX <= pose.getX() && pose.getX() <= maxX && minY <= pose.getY() && pose.getY() <= maxY)
        {
            return maxVelocity.get(s, pose, deriv, baseRobotVel);
        }
        else
        {
            return Double.POSITIVE_INFINITY;
        }
    }

}