package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardController;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.Estimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Mat;

import java.util.Collections;
import java.util.function.DoubleSupplier;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinner {

    public static double MAX_ANGLE = 60;
    public static double MIN_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static double power = 0.2;
    public static boolean usePID = true;
    BasicPID PID;
    AngleController controller;
    public static double kP = 2.7;
    public static double kI = 0;
    public static double kD = 0;
    public static double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    public DcMotorEx motor;
    PositionVelocitySystem positionVelocitySystem;

    DoubleSupplier positionSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return motor.getCurrentPosition();
        }
    };
    Estimator positionEstimator = new Estimator(positionSupplier) {
        @Override
        public double update() {
            return motor.getCurrentPosition();
        }
    };

    DoubleSupplier velocitySupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return motor.getVelocity();
        }
    };
    Estimator velocityEstimator = new Estimator(velocitySupplier) {
        @Override
        public double update() {
            return motor.getVelocity();
        }
    };

    double Kv = 1.1;
    double Ka = 0.2;
    double Ks = 0.001;
    FeedforwardCoefficients coefficientsKVA = new FeedforwardCoefficients(Kv,Ka,Ks);
    BasicFeedforward feedforwardController = new BasicFeedforward(coefficientsKVA);
    Vector K = new Vector(new double[] {1.1,0.3});
    FullStateFeedback controllerPOS = new FullStateFeedback(K);

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

    public ElevatorSpinner(HardwareMap hardwareMap, Gamepad gamepad)
    {
        PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
        controller = new AngleController(PID);
        motor = hardwareMap.get(DcMotorEx.class, "mS");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        positionVelocitySystem = new PositionVelocitySystem(positionEstimator, velocityEstimator,
                feedforwardController,
             controllerPOS);
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
            motor.setPower(controller.calculate(target, encoderTicksToRadians(motor.getCurrentPosition())));
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