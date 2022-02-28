/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand.HandPos;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;

import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL1;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL2;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.HUB_LEVEL3;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.ZERO_HEIGHT;
import static org.firstinspires.ftc.teamcode.robot.subsystems.Elevator.elevatorLevel;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MAX_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.MIN_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner.ZERO_ANGLE;

@Config
@TeleOp(group = "main")
public class teleopOwlBlue extends LinearOpMode {

    gamepad gamepad;
    //ElevatorSpinnerCOMPLEX_UNSTABLE spinner;
    ElevatorSpinner spinner;
    Elevator elevator;
    carousel carousel;
    //IntakeFixingThread intake;
    //Thread intakeFixingThread;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1;
    public static double powerIntake = 1;
    boolean canIntake = true;


    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }

    public static int elevatorLevel = 3;
    public static int spinnerLevel = 2;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        //spinner = new ElevatorSpinnerCOMPLEX_UNSTABLE(hardwareMap, gamepad2);
        spinner = new ElevatorSpinner(hardwareMap, gamepad2);
        elevator = new Elevator(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intake = new IntakeFixingThread(hardwareMap, telemetry);
        //intakeFixingThread = intake;
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);

        // wait till after init
        waitForStart();

        //intakeFixingThread.start();

        while (opModeIsActive()) {
            cGamepad1.update();

            gamepad.update();
            elevator.update();
            spinner.update();
            intakeToggles();
            elevatorSwitch();
            resetElevatorMidMoving();
            pidToggles();
            carouselSpinning();
            handMoving();
            elevatorLeveling();
            telemetry.update();
        }

        gamepad.saveIMUHeading();

        //intake.exitThread();
        //intakeFixingThread.interrupt();
    }

    void elevatorLeveling()
    {
        if(gamepad2.a)
        {
            elevatorLevel = 1;
        }
        else if(gamepad2.b)
        {
            elevatorLevel = 2;
        }
        else if(gamepad2.y)
        {
            elevatorLevel = 3;
        }
        else if(gamepad2.x)
        {
            elevatorLevel = 0;
        }
    }

    void handMoving()
    {
        hand.moveTo(gamepad2.right_trigger);
    }
    void pidToggles()
    {
        if(gamepad2.left_stick_button)
        {
            spinner.setUsePID(false);
            elevator.setUsePID(false);
        }
        else if(gamepad2.right_stick_button)
        {
            spinner.setUsePID(true);
            elevator.setUsePID(true);
        }
    }

    void carouselSpinning()
    {
        if(gamepad2.right_bumper)
        {
            carousel.spin(false);
        }
        else if(gamepad2.left_bumper)
        {
            carousel.spin(true);
        }
        else
        {
            carousel.stop();
        }
    }

    void intakeToggles()
    {
        if ((gamepad1.left_trigger != 0) && canIntake)
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad1.right_trigger != 0) && canIntake)
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
        }


        powerIntake();
    }

    void powerIntake()
    {
        if(frontIntake && canIntake)
        {
            intake.powerIntake(powerIntake);
        }
        else if(backIntake && canIntake)
        {
            intake.powerIntake(-powerIntake);
        }
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0)
        {
            intake.stop();
        }
    }

    void elevatorSwitch()
    {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.update();
                spinner.update();

                if (gamepad1.right_bumper)
                {
                    //intake.spinIntake = false;
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    hand.intake();
                    dip.holdFreight();

//                    switch (spinnerLevel)
//                    {
//                        case 0:
//                            spinner.setTarget(MIN_ANGLE);
//                            break;
//                        case 2:
//                            spinner.setTarget(MAX_ANGLE);
//                            break;
//                        default:
//                            spinner.setUsePID(false);
//                    }
//
                    switch (elevatorLevel)
                    {
                        case 0:
                            elevatorMovement = ElevatorMovement.SHARED;
                            break;
                        case 1:
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                        default:
                            elevator.setUsePID(false);
                    }


                }
                break;
            case LEVEL1:
                spinner.setSpinnerState(ElevatorSpinner.SpinnerState.LEFT);
                elevator.setElevatorLevel(Elevator.ElevatorLevel.HUB_LEVEL1);
                hand.level1();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                spinner.setSpinnerState(ElevatorSpinner.SpinnerState.LEFT);
                elevator.setElevatorLevel(Elevator.ElevatorLevel.HUB_LEVEL2);
                hand.level2();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                spinner.setSpinnerState(ElevatorSpinner.SpinnerState.LEFT);
                elevator.setElevatorLevel(Elevator.ElevatorLevel.HUB_LEVEL3);
                hand.level3();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                spinner.setSpinnerState(ElevatorSpinner.SpinnerState.RIGHT);
                elevator.setElevatorLevel(Elevator.ElevatorLevel.SHARED_HUB);
                hand.shared();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                if(elevator.getTarget() == SHARED_HUB)
                {
                    elevator.setElevatorLevel(Elevator.ElevatorLevel.ZERO);
                }

                elevator.update();
                spinner.update();

                if(gamepad1.left_bumper)
                {
                    dip.releaseFreight();
                    if(elevator.getTarget() == ZERO_HEIGHT)
                    {
                        elevator.setElevatorLevel(Elevator.ElevatorLevel.SHARED_HUB);
                    }
                    hand.intake();
                    //intake.spinIntake = true;
                    canIntake = true;
                    frontIntake = true;
                    elevatorMovement = ElevatorMovement.SPIN;
                }
                break;
            default:
                elevatorMovement = ElevatorMovement.SPIN;
                break;
        }
    }

    void resetElevator()
    {
        dip.getFreight();
        hand.intake();
        elevator.setElevatorLevel(Elevator.ElevatorLevel.ZERO);
        spinner.setSpinnerState(ElevatorSpinner.SpinnerState.ZERO);
        canIntake = true;
    }

    void resetElevatorMidMoving()
    {
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.SPIN)
        {
            elevatorMovement = ElevatorMovement.SPIN;
            //intake.spinIntake = true;
            canIntake = true;
            frontIntake = true;
        }

    }


}