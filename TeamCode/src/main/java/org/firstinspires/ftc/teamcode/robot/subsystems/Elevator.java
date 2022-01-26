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


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class Elevator {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 1150;

    public static double SPOOL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (spool) speed / input (motor) speed

    public static double MAX_HEIGHT = 20;
    public static double MID_HEIGHT = 13;
    public static double MIN_HEIGHT = 9;
    public static double ZERO_HEIGHT = 0;

    public static PIDCoefficients PID = new PIDCoefficients(2.5, 2, 0);

    public static double MAX_VEL = 80; // in/s
    public static double MAX_ACCEL = 80; // in/s^2
    public static double MAX_JERK = 200; // in/s^3

    public static double kV = 10;
    public static double kA = 10;
    public static double kStatic = 0;

    private DcMotorEx motor;
    private MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredHeight = 0;
    private int offset;
    PIDFController controller;
    double power;

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * SPOOL_RADIUS / 60.0;
    }

    public static double getMaxRpm() {
        return MAX_RPM;
    }

    public Elevator(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "mE");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // if necessary, reverse the motor so "up" is positive
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy() ? profile.get(time) : new MotionState(desiredHeight, 0, 0, 0);
        MotionState goal = new MotionState(height, 0, 0, 0);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start, goal, MAX_VEL, MAX_ACCEL, MAX_JERK
        );

        profileStartTime = clock.seconds();

        this.desiredHeight = height;
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double currentHeight = getCurrentHeight();
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            controller.setTargetPosition(state.getX());
            power = controller.update(currentHeight, state.getV());
        } else {
            // just hold the position
            controller.setTargetPosition(desiredHeight);
            power = controller.update(currentHeight);
        }
        setPower(power);
    }

    public double getVelocity()
    {
        if (isBusy()) {
            // following a profile
            double time = clock.seconds() - profileStartTime;
            MotionState state = profile.get(time);
            return state.getV();
        } else {
            return 0;
        }
    }

    public double getTargetVelocity()
    {
        return controller.getTargetVelocity();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }


}