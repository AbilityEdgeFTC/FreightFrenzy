package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint

class RectangleMask(
        val minX: Double, val minY: Double,
        val maxX: Double, val maxY: Double,
        val c: TrajectoryVelocityConstraint
) : TrajectoryVelocityConstraint {
    override fun get(s: Double, pose: Pose2d, deriv: Pose2d, baseRobotVel: Pose2d) =
            if (pose.x in minX..maxX && pose.y in minY..maxY) {
                c[s, pose, deriv, baseRobotVel]
            } else {
                Double.POSITIVE_INFINITY
            }
}

