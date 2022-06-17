package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightSensor;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
@TeleOp(name = "BLUE TeleOp - Driver Control", group = "RED")
public class teleOpBlue extends LinearOpMode {

    gamepad gamepad;
    SpinnerPID spinner;
    ElevatorFirstPID elevator;
    Carousel carousel;
    intake intake;
    hand hand;
    dip dip;
    Cover cover;
    cGamepad cGamepad1, cGamepad2;
    ElapsedTime resetElevator;
    FreightSensor freightSensor;

    public static double powerIntake = 1, powerSlowElevator = .65;
    public static double firstLevelHandDelay = 0.2, secondLevelHandDelay = .2;
    public static double thirdLevelHandDelay = .17, shareLevelHandDelay = 0.25;
    public static double spinnerSlowerPower = 0.4;
    public static double elevatorFastPower = 0.85;
    public static double sharedLevelElevatorGoBackDelay = 1, sharedLevelElevatorCloseDelay = 1;
    public static double closingHandDelayShare = .65, closingHandDelayLevel1 = 1.3;
    public static double closingHandDelayLevel2 = 1, closingHandDelayLevel3 = .43;
    double MIN_MANUAL_HAND_MOVING = 0.03, MAX_MANUAL_HAND_MOVING = 1 - MIN_MANUAL_HAND_MOVING;
    boolean frontIntake = false, backIntake = false, canIntake = true;
    public static double delayCloseCover = 1.15;
    double positionDip = 0;

    enum ElevatorMovement
    {
        CLOSED,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }

    int elevatorLevel = 3;
    public static ElevatorMovement elevatorMovement = ElevatorMovement.CLOSED;
    public static ElevatorMovement lastMovement = ElevatorMovement.CLOSED;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad.setRedAlliance(false);
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new SpinnerPID(hardwareMap, gamepad1, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new Carousel(hardwareMap , gamepad2);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cover = new Cover(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        freightSensor = new FreightSensor(hardwareMap);
        resetElevator = new ElapsedTime();

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {

            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();
            elevator.update();
            spinner.update();
            toggleIntakesGP1GP2();
            elevatorSwitch();
            resetElevatorMidMoving();
            checkIfUserWantsToTurnOffPIDWhileItsOn();
            manualServoMoving();
            switchElevatorLevelsGP2();
            spinCarousel();
            telemetry.update();

        }

        gamepad.saveIMUHeading();
    }

    /**
     * Function changes the state of carousel(on,off) by the dpad left and right.
     */
    void spinCarousel()
    {
        if (gamepad2.dpad_right)
        {
            carousel.spinCarousel(false);
            toggleIntakesGP1GP2();
        }
        else if(gamepad2.dpad_left)
        {
            carousel.spinCarousel( true);
            toggleIntakesGP1GP2();
        }
        else
        {
            carousel.stopCarousel();
        }
    }

    /**
     * Function gets a elevator level, and check if the current position of the elevator is greater than it's offset so that
     * means that the elevator is open, and we want to change it's current level.
     */
    void switchElevatorLevelIfIsOpen(ElevatorFirstPID.ElevatorLevel level)
    {
        if(elevator.getTargetPosition() > elevator.getOffset())
        {
            elevator.setElevatorLevel(level);
        }
    }

    /**
     * Function changes the target elevator level by user input, and if the elvator is open, it will change it's
     * current level.
     */
    void switchElevatorLevelsGP2()
    {
        // each button on game pad sets the target level of the elevator to a different level
        if(gamepad1.y)
        {
            elevatorLevel = 3;

            if(elevatorMovement != ElevatorMovement.CLOSED)
            {
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                hand.level3();
            }
        }
        else if(gamepad2.x)
        {
            elevatorLevel = 2;

            if(elevatorMovement != ElevatorMovement.CLOSED)
            {
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                hand.level2();
            }
        }
        else if (gamepad2.y)
        {
            elevatorLevel = 0;

            if(elevatorMovement != ElevatorMovement.CLOSED)
            {
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                hand.shared();
            }
        }
    }

    /**
     * Function lets the hand servo go up and down with user input on gamepad 2 bumbers
     */
    void moveHandServoManuallyGP2()
    {
        // get current pos
        double position = hand.getPos();

        // check if the user wants to make the hand go up or down, and it can(0-1 range)
        if(cGamepad2.rightBumperOnce() && hand.getPos() >= MIN_MANUAL_HAND_MOVING)
        {
            // if so, go up
            position -= MIN_MANUAL_HAND_MOVING;
            hand.moveTo(position);
        }
        else if(cGamepad2.leftBumperOnce() && hand.getPos() <= MAX_MANUAL_HAND_MOVING)
        {
            // else if so, go down
            position += MIN_MANUAL_HAND_MOVING;
            hand.moveTo(position);
        }
    }

    /**
     * FUNCTION CURRENTLY DISABLED!!!!
     * Function lets the dip servo go up and down with user input on gamepad 2 dpads up and down
     */
    void manual1ServoDipMoving()
    {
        // get current pos
        positionDip = dip.getPos();

        // check if the user wants to make the dip go up or down, and it can(0-1 range)
        /*if(cGamepad2.dpadDownOnce() && dip.getPos() >= 0.02)
        {
            // if so, go up
            positionDip -= 0.02;
            dip.moveTo(positionDip);
        }
        else if(cGamepad2.dpadUpOnce() && dip.getPos() <= 0.98)
        {
            // if so, go down
            positionDip += 0.02;
            dip.moveTo(positionDip);
        }*/

    }

    /**
     * Function turns off the pid for a temp time while user is with gamepad
     */
    boolean checkIfUserWantsToTurnOffPIDWhileItsOn()
    {
        // these are the spinner/elevator manual buttons, so we check if user has changed them(moved), if so turn off/turn on pid
        // for a temp time, else turn on pid.
        if(gamepad2.right_stick_x != 0 || gamepad2.right_stick_button && elevatorMovement == ElevatorMovement.CLOSED)
        {
            spinner.setUsePID(false);
            return false;
        }
        else if(elevatorMovement == ElevatorMovement.CLOSED)
        {
            spinner.setUsePID(true);
        }

        if(gamepad2.left_stick_y != 0 || gamepad2.left_stick_button && elevatorMovement == ElevatorMovement.CLOSED)
        {
            elevator.setUsePID(false);
            if(gamepad2.start)
            {
                elevator.resetElevator();
            }
            return false;
        }
        else if(elevatorMovement == ElevatorMovement.CLOSED && gamepad2.right_stick_x == 0)
        {
            elevator.setUsePID(true);
        }

        if((gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1) || gamepad1.left_bumper &&
                !gamepad2.left_stick_button && elevatorMovement != ElevatorMovement.CLOSED)
        {
            elevator.setUsePID(true);
            spinner.setUsePID(true);
        }

        return true;
    }

    /**
     * Function toggles the intake forward/backward/stop with the gamepads 1 and 2
     */
    void toggleIntakesGP1GP2()
    {
        /**
         theres alot of if's because we want to make sure there isn't any collusion between the intake when 2 drivers are trying to power
         it by accident...
         but we just check if the gamepads want to power the intake, and which way.
         */

        if ((gamepad1.right_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0) && (gamepad1.left_trigger == 0) && !freightSensor.hasFreight())
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad1.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad2.left_trigger == 0) && !freightSensor.hasFreight())
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.right_trigger != 0) && canIntake && (gamepad2.left_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) && !freightSensor.hasFreight())
        {
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) && !freightSensor.hasFreight())
        {
            intake.powerIntake(-gamepad2.left_trigger);
            frontIntake = false;
            backIntake = false;
        }

        if(freightSensor.hasFreight())
        {
            frontIntake = false;
            backIntake = true;
        }


        toggleIntake();
    }

    /**
     * Function toggles intake by the booleans with no need of holding buttons.
     */
    void toggleIntake()
    {
        if(frontIntake && canIntake)
        {
            intake.powerIntake(powerIntake);
        }
        else if(backIntake && canIntake)
        {
            intake.powerIntake(-powerIntake);
        }
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
        {
            intake.stop();
        }

    }

    /**
     * Function gets a position to move the dip, and if the pid is on so move to that position.
     * @param position the poisiton to move the dip
     */
    void moveAutomaticallyDip(double position)
    {
        if(!withoutPID())
        {
            dip.moveTo(position);
        }
    }

    /**
     * Function checks that the pid is off, and turns on manual hand and dip servo control.
     */
    void manualServoMoving()
    {
        if(withoutPID())
        {
            moveHandServoManuallyGP2();
            manual1ServoDipMoving();
        }
    }

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case CLOSED:
                // while elevator is closed, spinner manual moving should be regular, and the gamepad 1 should let the user
                // to spin the robot and not the spinner
                if(lastMovement == ElevatorMovement.SHARED)
                {
                    closeSharedHubElevator();
                }
                else
                {
                    resetElevator();
                }
                spinner.setUsePID(true);
                spinner.setSlowMove(false);
                gamepad.setCanTwist(true);

                // user wants to open elevator
                if (gamepad1.right_bumper)
                {
                    // turn on pid if it was off, spinner movement should be slower and the gamepad1 should twist the spinner
                    // not the robot
                    elevator.setUsePID(true);
                    spinner.setSlowMove(true);
                    spinner.setUsePID(false);
                    gamepad.setCanTwist(false);

                    // turn of intake, and the option for intake
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;

                    cover.openCover();

                    // go to the next state by the elevator level
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
                    }

                    resetElevator.reset();
                }
                break;
            case LEVEL1:
                lastMovement = ElevatorMovement.LEVEL1;
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1, firstLevelHandDelay, hand.getLevel1Hub());
                break;
            case LEVEL2:
                lastMovement = ElevatorMovement.LEVEL2;
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2, secondLevelHandDelay, hand.getLevel2Hub());
                break;
            case LEVEL3:
                lastMovement = ElevatorMovement.LEVEL3;
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3, thirdLevelHandDelay, hand.getLevel3Hub());
                break;
            case SHARED:
                lastMovement = ElevatorMovement.SHARED;
                // move elevator to the target, and add a delay for the opening of the hand
                sharedLevelElevatorControl();
                break;
            case DIP:
                // this will be in a loop, so update elevator and turn on manual servo moving
                elevator.update();
                manualServoMoving();

                // if gp1 right stick button is on, so we want to twist the robot and not spinner
                if(gamepad1.right_stick_button)
                {
                    gamepad.setCanTwist(true);
                    spinner.setSlowMove(false);
                    spinner.setUsePID(true);
                }
                // else we want to twist spinner not robot
                else
                {
                    gamepad.setCanTwist(false);
                    spinner.setUsePID(false);
                    spinner.setSlowMove(true);
                }

                switchElevatorLevelsGP2();

                if(gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
                {
                    // release freight
                    dip.releaseFreight();
                }

                // user wants to close elevator
                if(gamepad1.left_bumper)
                {
                    // release freight
                    dip.releaseFreight();

                    // make spinner slower and close elevator faster
                    spinner.setMaxPower(spinnerSlowerPower);
                    elevator.setMaxPower(elevatorFastPower);
                    elevator.setUsePID(true);

                    // save the current position of the elevator
                    saveCurrentHubLevel();

                    // reset elvator timer
                    resetElevator.reset();

                    // let the intake work and turn it on forwardly
                    canIntake = true;
                    frontIntake = true;

                    // go to the closing state
                    elevatorMovement = ElevatorMovement.CLOSED;
                }
                break;
            default:
                elevatorMovement = ElevatorMovement.CLOSED;
                break;
        }
    }

    /**
     * Function sees which level is the elevator now, and saves it that way the next time we go to that level, it goes to the
     * lastly closed position.
     */
    void saveCurrentHubLevel()
    {
        switch (elevatorLevel) {
            case 0:
                elevator.setSharedHub(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                hand.setLevelSharedHub(hand.getPos());
            case 1:
                //elevator.setHubLevel1(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                //hand.setLevel1Hub(hand.getPos());
                break;
            case 2:
                //elevator.setHubLevel2(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                //hand.setLevel2Hub(hand.getPos());
                break;
            case 3:
                elevator.setHubLevel3(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                hand.setLevel3Hub(hand.getPos());
                break;
        }
    }

    /**
     * Function seta new targel level, closes the dip and waits a delay till we move the hand
     * @param elevatorLevel new elevator level
     * @param delay the amount to wait before opening the hand
     * @param pos where should the hand move to
     */
    void setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel elevatorLevel, double delay, double pos)
    {
        // set new elevator level
        elevator.setElevatorLevel(elevatorLevel);

        // wait the delay(if with pid on ofc)
        if(!withoutPID() && resetElevator.seconds() > delay)
        {
            // move to closing dip position
            moveAutomaticallyDip(dip.getHoldingPosition());
            // move hand to the pos and go on to the next state in the finite state machine.
            hand.moveTo(pos);
            elevatorMovement = ElevatorMovement.DIP;
        }
    }

    void sharedLevelElevatorControl()
    {
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);

        // wait the delay(if with pid on ofc)
        if(!withoutPID() && resetElevator.seconds() > shareLevelHandDelay)
        {
            // move to closing dip position
            moveAutomaticallyDip(dip.getHoldingPosition());
            // move hand to the pos and go on to the next state in the finite state machine.
            hand.shared();
        }

        if(!withoutPID() && resetElevator.seconds() > shareLevelHandDelay + sharedLevelElevatorGoBackDelay)
        {
            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
            elevatorMovement = ElevatorMovement.DIP;
        }
    }

    void closeSharedHubElevator()
    {

        // wait the delay(if with pid on ofc)
        if(!withoutPID())
        {
            // move to closing dip position
            dip.getFreight();
        }

        if(resetElevator.seconds() > closingHandDelayShare / 2)
        {
            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
        }

        if(!withoutPID() && resetElevator.seconds() > closingHandDelayShare)
        {
            // move hand to the pos and go on to the next state in the finite state machine.
            hand.intake();

        }

        if(!withoutPID() && resetElevator.seconds() > closingHandDelayShare + sharedLevelElevatorCloseDelay)
        {
            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
            cover.closeCover();
            lastMovement = ElevatorMovement.CLOSED;
        }
    }

    /**
     * Function resets the dip, slowly closes the elevator, let's the intake to turn on, and closes the hand with/without delay.
     */
    void resetElevator()
    {
        // open dip
        moveAutomaticallyDip(dip.getReleasingPosition());

        // close slowly elevator
        slowCloseElevator();

        // close the hand with a delay
        switch (elevatorLevel)
        {
            case 0:
                delayEleavtorCloseAndReturnHand(closingHandDelayShare);
                break;
            case 1:
                delayEleavtorCloseAndReturnHand(closingHandDelayLevel1);
                break;
            case 2:
                delayEleavtorCloseAndReturnHand(closingHandDelayLevel2);
                break;
            case 3:
                delayFullRetractionOfElevator(closingHandDelayLevel3);
                break;
        }

        // robot can intake, and update the elevator
        elevator.update();
        canIntake = true;
    }

    /**
     * Function closes the elevator with a slower poer
     */
    void slowCloseElevator()
    {
        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
    }

    /**
     * Function returns delays the retraction of the hand to intake position, and elevator closing
     * @param delay the delay till the full elevator retraction
     */
    void delayFullRetractionOfElevator(double delay)
    {
        // wait delay long
        if(resetElevator.seconds() > delay && checkIfUserWantsToTurnOffPIDWhileItsOn())
        {
            // return hand
            hand.intake();

            // turn on pid
            elevator.setUsePID(true);
        }
        else
        {
            elevator.setUsePID(false);
        }

        if(resetElevator.seconds() > delayCloseCover && checkIfUserWantsToTurnOffPIDWhileItsOn())
        {
            cover.closeCover();
        }


    }

    /**
     * Function returns the hand to intake position, and adds a delay till the elevator will close
     * @param delay the delay till the elevator closes
     */
    void delayEleavtorCloseAndReturnHand(double delay)
    {
        // return hand to intake pos
        hand.intake();

        // check if delay has gone by, if yes turn on pid and return elevator, else, don't
        if(resetElevator.seconds() > delay && checkIfUserWantsToTurnOffPIDWhileItsOn())
        {
            elevator.setUsePID(true);
        }
        else
        {
            elevator.setUsePID(false);
        }

        if(resetElevator.seconds() > delayCloseCover && checkIfUserWantsToTurnOffPIDWhileItsOn())
        {
            cover.closeCover();
        }
    }

    /**
     * Check if the user wants the reset the elevator while it's moving in the finite state machine, if so
     * reset and turn on intake
     */
    void resetElevatorMidMoving()
    {
        // does user want to reset mid moving, and the elevator isn't closed?
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.CLOSED)
        {
            // reset elevator logic
            elevatorMovement = ElevatorMovement.CLOSED;

            // turn on intake
            canIntake = true;
            frontIntake = true;
        }

    }

    /**
     * Function check is elevator pid is on and at level 0, if so set the hand to the correct position and return false, else return true
     */
    boolean withoutPID()
    {
        if(elevator.getUsePID() == true && elevator.getElevatorLevel() != ElevatorFirstPID.ElevatorLevel.ZERO)
        {
            switch (hand.getHandPos())
            {
                case SHARED_HUB:
                    hand.shared();
                    break;
                case INTAKE:
                    break;
                case ONE_HUB:
                    hand.level1();
                    break;
                case TWO_HUB:
                    hand.level2();
                    break;
                case THREE_HUB:
                    hand.level3();
                    break;
            }
            return false;
        }
        else
        {
            return true;
        }
    }


}