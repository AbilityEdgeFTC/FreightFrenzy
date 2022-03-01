package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinnerMotion {

    public static final double MAX_RPM = 312;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    public static double SPOOL_RADIUS = 0.75; // in

    public static double RIGHT_ANGLE = 60;
    public static double LEFT_ANGLE = -60;
    public static double ZERO_ANGLE = 0;

    public static PIDCoefficients PID = new PIDCoefficients(2, 0, 0);

    public static double MAX_VEL = 10; // in/s
    public static double MAX_ACCEL = 5; // in/s^2
    public static double MAX_JERK = 2; // in/s^3

    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;

    public static DcMotorEx motor;
    public static MotionProfile profile;
    private NanoClock clock = NanoClock.system();
    private double profileStartTime, desiredAngle = 0;
    public static int offset;
    public static PIDFController controller;
    public static double power;

    private static double encoderTicksToRadians(int ticks) {
        return Math.toRadians((ticks * 360) / TICKS_PER_REV);
    }

    private static double radiansToEncoderTicks(double radians) {
        return TICKS_PER_REV / (radians * 2 * Math.PI);
    }

    public static double getMaxRpm() {
        return MAX_RPM;
    }

    public ElevatorSpinnerMotion(HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward
        controller = new PIDFController(PID, kV, kA, kStatic);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy(boolean setTrue) {
        if(setTrue)
        {
            return true;
        }
        else
        {
            return profile != null && (clock.seconds() - profileStartTime) <= profile.duration();
        }
    }

    public void setAngle(double angle) {
        angle = Math.min(Math.max(LEFT_ANGLE, angle), RIGHT_ANGLE);
        angle = Angle.normDelta(Math.toRadians(angle));

        double time = clock.seconds() - profileStartTime;
        MotionState start = isBusy(false) ? profile.get(time) : new MotionState(desiredAngle, 0, 0, 0);
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
        double currentAngle = getCurrentAngle();
        if (isBusy(false)) {
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

    public double getVelocity()
    {
        if (isBusy(false)) {
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

    public double getPower() {
        return motor.getPower();
    }



}