package org.firstinspires.ftc.teamcode.robot.subsystems;
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
public class ElevatorCOMPLEX_UNSTABLE {

    public static double HUB_LEVEL1 = 22,HUB_LEVEL2 = 19,HUB_LEVEL3 = 17;
    public static double SHARED_HUB = 4.5;
    public static double ZERO_HEIGHT = 0;
    DcMotorEx motor;
    BasicPID controller;
    double target = 0;
    public static double TICKS_PER_REV = 384.5;
    public static double SPOOL_RADIUS = 0.75; // in
    public static boolean usePID = true;
    public static double MAX_VEL = 20;
    public static double MAX_ACCEL = 10;
    public static double kPPOS = .05;
    public static double kIPOS = 0;
    public static double kDPOS = 0;
    public static double kPVEL = 1;
    public static double kIVEL = 0;
    public static double kDVEL = 0;
    public static double kV = 0;
    public static double kA = 0;
    public static double kS = 0;
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


    public enum ElevatorLevel {
        ZERO,
        SHARED_HUB,
        HUB_LEVEL1,
        HUB_LEVEL2,
        HUB_LEVEL3
    }

    public static ElevatorLevel elevatorLevel = ElevatorLevel.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;
    PositionVelocitySystem system;

    public ElevatorCOMPLEX_UNSTABLE(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mE");
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        system =
                new PositionVelocitySystem(positionFilter,
                        velocityFilter,feedforward,controller,veloControl);
        this.gamepad = gamepad;
        this.cGamepad = new cGamepad(gamepad);
    }

    public void update()
    {
        if (usePID)
        {
            switch (elevatorLevel)
            {
                case ZERO:
                    target = ZERO_HEIGHT;
                    break;
                case SHARED_HUB:
                    target = SHARED_HUB;
                    break;
                case HUB_LEVEL1:
                    target = HUB_LEVEL1;
                    break;
                case HUB_LEVEL2:
                    target = HUB_LEVEL2;
                    break;
                case HUB_LEVEL3:
                    target = HUB_LEVEL3;
                    break;

            }
            motor.setPower(system.update(inchesToEncoderTicks(target), MAX_VEL, MAX_ACCEL));
        }
        else
        {
            motor.setPower(gamepad.left_stick_y);

        }
    }

    public static double encoderTicksToInches(int ticks) {
        return SPOOL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int inchesToEncoderTicks(double inches) {
        return (int)Math.round((inches * TICKS_PER_REV) / (SPOOL_RADIUS * 2 * Math.PI));
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