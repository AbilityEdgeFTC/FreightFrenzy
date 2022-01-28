package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.opencv.core.Mat;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class myElevator {
    public static final double TICKS_PER_REV = 145.1;
    public static final double MAX_RPM = 1150;
    public static double SPOOL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (spool) speed / input (motor) speed
    boolean moveToMin = false, moveToMid = false, moveToMax = false, moveToZero = false;
    public static double MAX_HEIGHT = 20;
    public static double MID_HEIGHT = 13;
    public static double MIN_HEIGHT = 9;
    public static double ZERO_HEIGHT = 0;

    public static PIDFCoefficients PIDF = new PIDFCoefficients(0, 0, 0, 0);

    PIDFController controller;
    private DcMotorEx motor;
    private int offset;

    private static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private static double inchesToEncoderTicks(double inches) {
        return (inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI * GEAR_RATIO);
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
        //controller = new PIDFController(PIDF.p, PIDF.i, PIDF.d);
        controller = new PIDFController(PIDF.p, PIDF.i, PIDF.d, PIDF.f);
        offset = motor.getCurrentPosition();
    }

    public boolean isBusy() {
        return motor.isBusy();
    }

    public void setHeight(double height) {
        height = Math.min(Math.max(0, height), MAX_HEIGHT);
        controller.setSetPoint(inchesToEncoderTicks(height));
    }

    public void goToZero()
    {
        setHeight(ZERO_HEIGHT);
        moveToMid = false;
        moveToMin = false;
        moveToMax = false;
        moveToZero = true;
    }

    public void goToMin()
    {
        setHeight(MIN_HEIGHT);
        moveToMid = false;
        moveToMin = true;
        moveToMax = false;
        moveToZero = false;
    }

    public void goToMid()
    {
        setHeight(MID_HEIGHT);
        moveToMid = true;
        moveToMin = false;
        moveToMax = false;
        moveToZero = false;
    }

    public void goToMax()
    {
        setHeight(MAX_HEIGHT);
        moveToMid = false;
        moveToMin = false;
        moveToMax = true;
        moveToZero = false;
    }

    public double getCurrentHeight() {
        return encoderTicksToInches(motor.getCurrentPosition() - offset);
    }

    public void update() {
        double currentHeight = motor.getCurrentPosition();
        setVelocity(controller.calculate(currentHeight));

//        double currentHeight = getCurrentHeight();
//        power = controller.updatedPower(desiredHeight, currentHeight);
//        setPower(power);
    }

    public double getVelocity()
    {
        return motor.getVelocity();
    }

    public double getErrorVelocity()
    {
        return controller.getVelocityError();
    }

    public void stop(){
        setVelocity(0);
    }

    public void setVelocity(double power) {
        motor.setVelocity(power);
    }


}