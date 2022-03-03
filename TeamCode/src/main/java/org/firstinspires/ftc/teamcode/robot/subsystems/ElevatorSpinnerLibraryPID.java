package org.firstinspires.ftc.teamcode.robot.subsystems;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


/*
 * Hardware class for an elevator or linear lift driven by a pulley system.
 */
@Config
public class ElevatorSpinnerLibraryPID {

    public static double RIGHT_ANGLE = 60;
    public static double LEFT_ANGLE = -60;
    public static double ZERO_ANGLE = 0;
    public static double power = 0.2;
    public static boolean usePID = true;
    public static double kP = 2.5;
    public static double kI = 0;
    public static double kD = 0;
    double target = 0;
    public static double GEAR_RATIO = 146.0/60.0; // in
    public static double TICKS_PER_REV = 537.7 * GEAR_RATIO;
    DcMotorEx motor;
    BasicPID PID = new BasicPID(new PIDCoefficients(kP, kI, kD));
    AngleController controller = new AngleController(PID);

//    public static double kP = 2.5;
//    public static double kV = 0;
//    Vector K = new Vector(new double[] {kP,kV});
//    FullStateFeedback controller = new FullStateFeedback(K);
//    public static double MAX_VEL = 5;

    public enum SpinnerState
    {
        LEFT,
        ZERO,
        RIGHT
    }

    public static SpinnerState spinnerState = SpinnerState.ZERO;

    Gamepad gamepad;
    cGamepad cGamepad;

    public ElevatorSpinnerLibraryPID(HardwareMap hardwareMap, Gamepad gamepad)
    {
        this.motor = hardwareMap.get(DcMotorEx.class, "mS");
        this.motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
                    target = Math.toRadians(LEFT_ANGLE);
                    break;
                case RIGHT:
                    target = Math.toRadians(RIGHT_ANGLE);
                    break;
            }

//            // measured position and velocity
//            Vector state = new Vector(new double[]{motor.getCurrentPosition(), motor.getVelocity()});
//            // target position and velocity
//            // protip - these can be generated with motion profiles!
//            Vector reference = new Vector(new double [] {radiansToEncoderTicks(target), MAX_VEL});
//            motor.setPower(controller.calculate(reference,state));

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

    public static SpinnerState getSpinnerState() {
        return spinnerState;
    }

    public static void setSpinnerState(SpinnerState spinnerState) {
        ElevatorSpinnerLibraryPID.spinnerState = spinnerState;
    }
}