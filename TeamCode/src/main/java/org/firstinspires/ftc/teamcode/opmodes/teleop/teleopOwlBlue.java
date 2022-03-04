/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
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
import static org.firstinspires.ftc.teamcode.robot.subsystems.hand.intakePos;

@Config
@TeleOp(group = "main")
public class teleopOwlBlue extends LinearOpMode {

    gamepad gamepad;
    //ElevatorSpinnerCOMPLEX_UNSTABLE spinner;
    SpinnerFirstPID spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    //IntakeFixingThread intake;
    //Thread intakeFixingThread;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .4, powerElevator =.95, delayForHandInShared = 3;
    boolean canIntake = true;
    double position = 0;
    NanoClock clock = NanoClock.system();

    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }
    double startTime = 0, startSharedTime = 0;

    public static int elevatorLevel = 3;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        //spinner = new ElevatorSpinnerCOMPLEX_UNSTABLE(hardwareMap, gamepad2);
        spinner = new SpinnerFirstPID(hardwareMap, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intake = new IntakeFixingThread(hardwareMap, telemetry);
        //intakeFixingThread = intake;
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO);

        // wait till after init
        waitForStart();

        startTime = clock.seconds();

        //intakeFixingThread.start();

        while (opModeIsActive()) {
            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();
            elevator.update();
            spinner.update();
            intakeToggles();
            elevatorSwitch();
            resetElevatorMidMoving();
            pidToggles();
            carouselSpinning();
//            if(!canHand())
//            {
//                handMoving();
//            }
            elevatorLeveling();
            telemetry.update();
        }

        gamepad.saveIMUHeading();

        //intake.exitThread();
        //intakeFixingThread.interrupt();
    }

    void elevatorLeveling()
    {
        /*if(gamepad2.a)
        {
            elevatorLevel = 1;
        }
        else if(gamepad2.b)
        {
            elevatorLevel = 2;
        }
        else */if(gamepad2.y)
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
        if(cGamepad2.dpadUpOnce() && hand.getPos() <= 0.9)
        {
            position += 0.1;
            hand.moveTo(position);
        }
        else if(cGamepad2.dpadDownOnce() && hand.getPos() >= 0.1)
        {
            position -= 0.1;
            hand.moveTo(position);
        }
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

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.update();
                spinner.update();

                if (gamepad1.right_bumper)
                {
                    elevator.setPower(powerElevator);
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
                            elevatorMovement = ElevatorMovement.SHARED;
                            break;
                        case 1:
                            if(canHand())
                            {
                                hand.level1();
                            }
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            if(canHand())
                            {
                                hand.level2();
                            }
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            if(canHand())
                            {
                                hand.level3();
                            }
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                        default:
                            elevator.setUsePID(false);
                    }
                }
                break;
            case LEVEL1:
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                if(canHand())
                {
                    hand.shared();
                }
                startSharedTime = clock.seconds();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                if(elevator.getTarget() == SHARED_HUB && (startTime - startSharedTime) > delayForHandInShared)
                {
                    elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                }

                elevator.update();
                spinner.update();

                if(gamepad1.left_bumper)
                {
                    dip.releaseFreight();
                    if(elevator.getTarget() == ZERO_HEIGHT)
                    {
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
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
        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
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
        //return !(spinner.getUsePID() || elevator.getUsePID());
        return true;
    }


}