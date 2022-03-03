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

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorLibraryPID.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorLibraryPID.ZERO_HEIGHT;

@Config
@TeleOp(group = "main")
public class teleopOwlRed extends LinearOpMode {

    gamepad gamepad;
    //ElevatorSpinnerCOMPLEX_UNSTABLE spinner;
    SpinnerFirstPID spinner;
    ElevatorLibraryPID elevatorLibraryPID;
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

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        //spinner = new ElevatorSpinnerCOMPLEX_UNSTABLE(hardwareMap, gamepad2);
        spinner = new SpinnerFirstPID(hardwareMap, gamepad2);
        elevatorLibraryPID = new ElevatorLibraryPID(hardwareMap, gamepad2);
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
            elevatorLibraryPID.update();
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
        if(gamepad2.right_trigger != 0)
        {
            hand.moveTo(gamepad2.right_trigger);
        }
    }
    void pidToggles()
    {
        if(gamepad2.left_stick_button)
        {
            spinner.setUsePID(false);
            elevatorLibraryPID.setUsePID(false);
        }
        else if(gamepad2.right_stick_button)
        {
            spinner.setUsePID(true);
            elevatorLibraryPID.setUsePID(true);
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

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevatorLibraryPID.update();
                spinner.update();

                if (gamepad1.right_bumper)
                {
                    //intake.spinIntake = false;
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    if(canHand())
                    {
                        hand.intake();
                    }
                    dip.holdFreight();

                    switch (elevatorLevel)
                    {
                        case 0:
                            elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.SHARED_HUB);
                            spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                            elevatorMovement = ElevatorMovement.SHARED;
                            break;
                        case 1:
                            elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.HUB_LEVEL2);
                            spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.HUB_LEVEL1);
                            spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.HUB_LEVEL3);
                            spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                        default:
                            elevatorLibraryPID.setUsePID(false);
                    }
                }
                break;
            case LEVEL1:
                if(canHand())
                {
                    hand.level1();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                if(canHand())
                {
                    hand.level2();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                if(canHand())
                {
                    hand.level3();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                if(canHand())
                {
                    hand.shared();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                if(elevatorLibraryPID.getTarget() == SHARED_HUB)
                {
                    elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.ZERO);
                }

                elevatorLibraryPID.update();
                spinner.update();

                if(gamepad1.left_bumper)
                {
                    dip.releaseFreight();
                    if(elevatorLibraryPID.getTarget() == ZERO_HEIGHT)
                    {
                        elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.SHARED_HUB);
                    }
                    if(canHand())
                    {
                        hand.intake();
                    }
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
        if(canHand())
        {
            hand.intake();
        }
        elevatorLibraryPID.setElevatorLevel(ElevatorLibraryPID.ElevatorLevel.ZERO);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO);
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

    boolean canHand()
    {
        if(gamepad2.right_trigger != 0)
        {
            return false;
        }

        return true;
    }


}