package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.jetbrains.annotations.NotNull;

public class RectangleMaskConstraintAcceleration implements TrajectoryAccelerationConstraint {
    double minX, minY, maxX, maxY;
    TrajectoryAccelerationConstraint maxAccel;

    public RectangleMaskConstraintAcceleration(double minX, double minY, double maxX, double maxY, TrajectoryAccelerationConstraint maxAccel) {
        this.minX = minX;
        this.minY = minY;
        this.maxX = maxX;
        this.maxY = maxY;
        this.maxAccel = maxAccel;
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
            return maxAccel.get(s, pose, deriv, baseRobotVel);
        }
        else
        {
            return Double.POSITIVE_INFINITY;
        }
    }
}