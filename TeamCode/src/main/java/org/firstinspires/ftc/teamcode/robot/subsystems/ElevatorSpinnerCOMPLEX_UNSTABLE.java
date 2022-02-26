package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinnerCOMPLEX_UNSTABLE {

    public static double MAX_VEL = 10;
    public static double MAX_ACCEL = 10;
    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static boolean usePID = true;
    AngleController controller;
    public static double kPPOS = 2.7;
    public static double kIPOS = 0;
    public static double kDPOS = 0;
    public static double kPVEL = 1;
    public static double kIVEL = 0;
    public static double kDVEL = 0;
    public static double kV = 0;
    public static double kA = 0;
    public static double kS = 0;
    double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotorEx motor;

    public static double Q = 0.3;
    public static double R = 3;
    public static int N = 3;
    PIDCoefficients posCoefficients = new PIDCoefficients(kPPOS,kIPOS,kDPOS);
    PIDCoefficients veloCoefficients = new PIDCoefficients(kPVEL,kIVEL,kDVEL);
    BasicPID posControl = new BasicPID(posCoefficients);
    BasicPID veloControl = new BasicPID(veloCoefficients);

    DoubleSupplier motorPosition = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return motor.getCurrentPosition();
        }
    };
    DoubleSupplier motorVelocity = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return motor.getVelocity();
        }
    };

    KalmanEstimator positionFilter = new KalmanEstimator(motorPosition,Q,R,N);
    KalmanEstimator velocityFilter = new KalmanEstimator(motorVelocity,Q,R,N);

    FeedforwardCoefficients coefficientsFF = new FeedforwardCoefficients(kV,kA,kS);
    BasicFeedforward feedforward = new BasicFeedforward(coefficientsFF);

    public static int spinnerLevel = 0;
    public enum SpinnerState
    {
        LEFT,
        ZERO,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;
    PositionVelocitySystem system;

    public ElevatorSpinnerCOMPLEX_UNSTABLE(HardwareMap hardwareMap, Gamepad gamepad)
    {
        controller = new AngleController(posControl);
        motor = hardwareMap.get(DcMotorEx.class, "mS");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        system =
                new PositionVelocitySystem(positionFilter,
                        velocityFilter,feedforward,controller,veloControl);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        cGamepad.update();

        if(usePID)
        {
            switch (spinnerState)
            {
                case ZERO:
                    target = Math.toRadians(ZERO_ANGLE);
                    break;
                case LEFT:
                    target = Math.toRadians(MIN_ANGLE);
                    break;
                case RIGHT:
                    target = Math.toRadians(MAX_ANGLE);
                    break;
            }

            motor.setPower(system.update(radiansToEncoderTicks(target), MAX_VEL, MAX_ACCEL));
        }
        else
        {
            motor.setPower(gamepad.right_stick_x);
        }

    }

    public static double encoderTicksToRadians(int ticks) {
        return Math.toRadians((ticks * 360) / TICKS_PER_REV);
    }

    public static double radiansToEncoderTicks(double radians) {
        return TICKS_PER_REV / (radians * 2 * Math.PI);
    }

    public int getPosition()
    {
        return motor.getCurrentPosition();
    }

    public double getTarget()
    {
        return target;
    }

    public void setTarget(double newTarget)
    {
        target = newTarget;
    }

    public void setUsePID(boolean usePID)
    {
        this.usePID = usePID;
    }



}