package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Hardware class for a rotary arm (for linearly-actuated mechanisms, see Elevator).
 */
@Config
public class SpinnerMotionProfile {
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;

    public static double RIGHT_ANGLE = 60;
    public static double LEFT_ANGLE = -60;
    public static double ZERO_ANGLE = 0;

    // the operating range of the arm is [0, MAX_ANGLE]
    double MAX_ANGLE = Math.toRadians(RIGHT_ANGLE); // rad

    public static PIDCoefficients PID = new PIDCoefficients(0, 0, 0);

    public static double MAX_VEL = Math.PI / 8; // rad/s
    public static double MAX_ACCEL = Math.PI / 8; // rad/s^2
    public static double MAX_JERK = Math.PI / 8; // rad/s^3

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;


    private DcMotorEx motor;
    private PIDFController controller;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredAngle = 0;
    private int offset;


    private static double encoderTicksToRadians(int ticks) {
        return 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI / 60.0;
    }

    public SpinnerMotionProfile(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "mS");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // if necessary, reverse the motor so CCW is positive
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // note: if the arm is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it here (assuming no velocity PID) like so:
        // e.g., controller = new PIDFController(PID, kV, kA, kStatic,
        //                                       angle -> kA * 9.81 * Math.sin(angle - VERT_ANGLE));
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setAngle(double angle) {
        angle = Math.min(Math.max(LEFT_ANGLE, angle), MAX_ANGLE);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredAngle, 0, 0, 0);
        MotionState goal = new MotionState(angle, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );
        profileStartTime = clock.seconds();

        this.desiredAngle = angle;
    }

    public double getCurrentAngle() {
        return encoderTicksToRadians(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double power;
        double currentAngle = getCurrentAngle();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentAngle, state.getV());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredAngle);
            power = controller.update(currentAngle);
        }
        setPower(power);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

}