/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
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
@TeleOp(name = "TeleOp Red Alliance", group = "main")
public class teleopOwlRed extends LinearOpMode {

    gamepad gamepad;
    //SpinnerFirstPID spinner;
    ElevatorSpinnerLibraryPID spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    //IntakeFixingThread intake;
    //Thread intakeFixingThread;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1, powerSlowElevatorNumber = .6;
    boolean canIntake = true;
    double position = 0;

    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }
    //ElapsedTime sharedTimer = new ElapsedTime();
    //boolean lastShared = false;

    public static int elevatorLevel = 3;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        //spinner = new SpinnerFirstPID(hardwareMap, gamepad2);
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        //intake = new IntakeFixingThread(hardwareMap, telemetry);
        //intakeFixingThread = intake;
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.ZERO_RED);

        // wait till after init
        waitForStart();

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
        if ((gamepad1.right_trigger != 0) && canIntake)
        {
            intake.powerIntake(-gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad1.left_trigger != 0) && canIntake)
        {
            intake.powerIntake(gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.right_trigger != 0) && canIntake)
        {
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.left_trigger != 0) && canIntake)
        {
            intake.powerIntake(-gamepad2.left_trigger);
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
                    powerSlowElevator = powerSlowElevatorNumber;
                    elevator.setPower(powerElevator);
                    //intake.spinIntake = false;
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    dip.holdFreight();

                    switch (elevatorLevel)
                    {
                        case 0:
                            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
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
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case SHARED:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                if(canHand())
                {
                    hand.shared();
                }
                //sharedTimer.reset();
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                elevator.update();
                spinner.update();

                /*if (sharedTimer.seconds() > delayForHandInShared) {
                    elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                }*/

                if(gamepad1.left_bumper)
                {

                    if(elevator.getTarget() == SHARED_HUB)
                    {
                        powerSlowElevatorNumber /= 2;
                    }

                    dip.releaseFreight();

                    if(canHand())
                    {
                        hand.intake();
                    }

                    elevator.setUsePID(true);
                    spinner.setUsePID(true);


                    /*if (gamepad1.x)
                    {
                        if(!lastShared)
                        {
                            sharedTimer.reset();
                        }
                        lastShared = true;
                    }
                    else
                    {
                        lastShared = false;
                    }

                    if (sharedTimer.seconds() > delayForHandInShared && canHand() && elevator.getTarget() == SHARED_HUB) {
                        hand.intake();
                    }*/


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
        switch (elevatorLevel)
        {
            case 0:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.LEFT);
                break;
            case 3:
            case 2:
            case 1:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                break;
        }
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