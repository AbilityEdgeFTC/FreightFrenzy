package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.acmerobotics.roadrunner.util.Angle.normDelta;

@Config
public class ElevatorController {

    public static double TICKS_PER_REV = 537.7;
    public static double SPOOL_RADIUS = 0.75; // in
    DcMotorEx motor;
    public static PIDCoefficients coefficients;
    double encoderPosition = 0;
    double error = 0, derivative, lastError, integralSum, errorChange, previousFilterEstimate = 0, currentFilterEstimate = 0, lastTarget;
    public static double integralSumLimit = 0.25, a = 0.8;

    public ElevatorController(HardwareMap hardwareMap, PIDCoefficients coefficients) {
        this.coefficients = coefficients;
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @return the command to our motor, I.E. motor power
     */
    public void updatePosition(double target, ElapsedTime time)
    {
        // obtain the encoder position
        encoderPosition = motor.getCurrentPosition();
        // calculate the error
        error = inchesToEncoderTicks(target) - encoderPosition;

        errorChange = (error - lastError);

        // filter out hight frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        derivative = (error - lastError) / time.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * time.seconds());

        // max out integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }

        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }

        // reset integral sum upon setpoint changes
        if (inchesToEncoderTicks(target) != lastTarget) {
            integralSum = 0;
        }

        motor.setPower((coefficients.kP * error) + (coefficients.kI * integralSum) + (coefficients.kD * derivative));

        lastError = error;
        lastTarget = inchesToEncoderTicks(target);

        // reset the timer for next time
        time.reset();
        // PID logic and then return the output
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @return the command to our motor, I.E. motor power
     */
    public void updateAngle(double target, ElapsedTime time)
    {
        double correctedTarget = normDelta(target);
        // obtain the encoder position
        encoderPosition = ticksToAngle(motor.getCurrentPosition());
        // calculate the error
        error = correctedTarget - encoderPosition;

        errorChange = (error - lastError);

        // filter out hight frequency noise to increase derivative performance
        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        // rate of change of the error
        derivative = (error - lastError) / time.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * time.seconds());

        // max out integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }

        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }

        // reset integral sum upon setpoint changes
        if (correctedTarget != lastTarget) {
            integralSum = 0;
        }

        motor.setPower((coefficients.kP * error) + (coefficients.kI * integralSum) + (coefficients.kD * derivative));

        lastError = error;
        lastTarget = correctedTarget;

        // reset the timer for next time
        time.reset();
        // PID logic and then return the output
    }

    public double getCurrentHeight()
    {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double angleToTicks(double angle){
        return TICKS_PER_REV / (360 / angle);
    }

    public double ticksToAngle(int ticks){
        return ticks / (TICKS_PER_REV / 360);
    }

    public static double encoderTicksToInches(double ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static double inchesToEncoderTicks(double inches) {
        return (inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI);
    }
}
