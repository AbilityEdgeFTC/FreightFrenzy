package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * How to tune:
 * 1. Start with Kp, Ki, and Kd at 0.
 * 2. Increase Kp until steady state error is very low.
 * 3. Increase Ki until steady state error is removed entirely.
 * 4. Increase Kd until oscillations are removed.
 *
 *
 *
 * Parameter   Rise time        Overshoot   Settling time   Steady-state error         Stability
 * Kp          Decrease         Increase    Small change    Decrease                   Degrade
 * Ki          Decrease         Increase    Increase        Eliminate                  Degrade
 * Kd          Little change    Decrease    Decrease        Theoretically no change    Improve if Kd is already low
 */
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDController {

    double Kp, Ki, Kd;
    double error = 0, derivative = 0, integralSum = 0, lastError = 0, max_i = 0.5;
    Telemetry telemetry;

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     * @param telemetry dashboard telemetry
     */
    public PIDController(double Kp, double Ki, double Kd, Telemetry telemetry) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.telemetry = telemetry;

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double updatedPower(double target, double state) {

        ElapsedTime timer = new ElapsedTime();

        // calculate the error
        error = target - state;

        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum += (error * (timer.seconds()));
        if(integralSum > max_i)
        {
            integralSum = max_i;
        }
        else if(integralSum < max_i)
        {
            integralSum = -max_i;
        }

        if(telemetry != null)
        {
            telemetry.addData("targetPosition", target);
            telemetry.addData("measuredPosition", state);
            telemetry.addData("error", error);
            telemetry.update();
        }

        lastError = error;
        // reset the timer for next time
        timer.reset();

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}