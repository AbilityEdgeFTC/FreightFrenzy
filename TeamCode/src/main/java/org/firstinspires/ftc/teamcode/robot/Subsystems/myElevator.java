package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class myElevator {
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;
    public static double SPOOL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (spool) speed / input (motor) speed

    public static double MAX_HEIGHT = 20;
    public static double MID_HEIGHT = 13;
    public static double MIN_HEIGHT = 9;
    public static double ZERO_HEIGHT = 0;

    public static PIDCoefficients PID = new PIDCoefficients(2.5, 2, 0);

    private DcMotorEx motor;
    private double desiredHeight = 0;
    private int offset;
    PIDController controller;
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

    public myElevator(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "mE");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // note: if the elevator is affected by a non-negligible constant force along the direction
        // of motion (e.g., gravity, kinetic friction, or a combination thereof), it may be
        // beneficial to compensate for it with a gravity feedforward
        controller = new PIDController(PID.kP, PID.kI, PID.kD);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);

        this.desiredHeight = height;
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double currentHeight = getCurrentHeight();
        power = controller.updatedPower(desiredHeight, currentHeight);
        setPower(power);
    }

    public double getVelocity()
    {
        return motor.getVelocity();
    }

    public double getError()
    {
        return controller.error;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }


}