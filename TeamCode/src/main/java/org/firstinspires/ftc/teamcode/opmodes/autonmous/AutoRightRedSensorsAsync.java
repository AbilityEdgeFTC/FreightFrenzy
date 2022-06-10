package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

import java.util.Arrays;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Right Red OLD ASYNC", group = "Autonomous Red")
public class AutoRightRedSensorsAsync extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -72 + 17.72;
    double startPoseRightH = 90;

    public static double poseEntranceX = 11.5;
    public static double poseEntranceFirstY = -61.5;
    public static double poseEntranceY = -66;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 60;
    public static double poseCollectY = -60;
    public static double poseCollectH = 180;
    public static double poseHelpX = 7;
    public static double poseHelpY = -55;
    public static double poseHelpH = 180;
    public static double poseIntakeFifteenX = 70;
    public static double poseIntakeFifteenY = -57;
    public static double poseIntakeFifteenH = 15;
    public static double poseIntakeThirtyH = 30;

    public static double GO_PARK_AT = 28;
    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1;
    public static double elevatorDelay = 1;
    double whiteLineX = 28.5;

    double emptyBox = 0;

    ModernRoboticsI2cRangeSensor rangeSensor;

    int wentIntakeXTimes = 0;
    boolean hasFreight = false;
    double offset = 0;
    boolean firstTime = true;
    boolean whiteLineDetected=false;
    SampleMecanumDrive drive;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    dip dip;
    intake intake;
    Cover cover;
    SensorColor colorSensor;
    ElapsedTime runningFor;

    TrajectorySequence fixAngle, goToHubFirst, goToHub, straightLineIntake, fifteenDegreeIntake, thirtyDegreeIntake, park;
    Pose2d startPoseRight, poseHelp, poseEntranceFirst, poseEntrance, poseCollect, poseGoToIntakeFifteen, poseGoToIntakeThirty;

    enum State
    {
        FIX_ANGLE, // fix angle from start angle
        INTAKE, // go to intake
        IDLE, // nothing
        LEAVE_EVERYTHING_AND_PARK,
        OPEN_ELEVATOR,
        WAIT_ELEVATOR_DELAY
    }

    enum intakePathType
    {
        STRAIGHT_PATH,
        FIFTEEN_DEGREE_PATH,
        THIRTY_DEGREE_PATH
    }

    State currentState = State.IDLE;
    intakePathType intakeType = intakePathType.STRAIGHT_PATH;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveConstants.setMaxVel(65);
        DriveConstants.setMaxAccel(40);

        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        hand = new hand(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        cover = new Cover(hardwareMap);
        colorSensor = new SensorColor(hardwareMap);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "dsC");

        startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        poseEntranceFirst = new Pose2d(poseEntranceX, poseEntranceFirstY, Math.toRadians(poseEntranceH));
        poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        poseGoToIntakeFifteen = new Pose2d(poseIntakeFifteenX, poseIntakeFifteenY, Math.toRadians(poseIntakeFifteenH));
        poseGoToIntakeThirty = new Pose2d(poseIntakeFifteenX, poseIntakeFifteenY, Math.toRadians(poseIntakeThirtyH));

        drive.setPoseEstimate(startPoseRight);

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeBackward();
            }
        };

        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
            }
        };

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(66),
                new RectangleMaskConstraint(45,-72,72,-45,
                        new TranslationalVelocityConstraint(8))));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(40);

        fixAngle = drive.trajectorySequenceBuilder(startPoseRight)
                .lineToLinearHeading(poseHelp, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL/3, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        goToHubFirst = drive.trajectorySequenceBuilder(fixAngle.end())
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntranceFirst, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        goToHub = drive.trajectorySequenceBuilder(fixAngle.end())
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        straightLineIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .build();

        fifteenDegreeIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .splineTo(new Vector2d(poseGoToIntakeFifteen.getX(), poseGoToIntakeFifteen.getY()), poseGoToIntakeFifteen.getHeading())
                .lineToSplineHeading(poseCollect)
                .strafeLeft(2)
                .build();

        thirtyDegreeIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .splineTo(new Vector2d(poseGoToIntakeThirty.getX(), poseGoToIntakeThirty.getY()), poseGoToIntakeThirty.getHeading())
                .lineToSplineHeading(poseCollect)
                .strafeLeft(2)
                .build();

        park = drive.trajectorySequenceBuilder(goToHub.end())
                .addTemporalMarker(intakeStop)
                .lineToSplineHeading(poseCollect)
                .build();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        dip.getFreight();
        cover.closeCover();

        waitForStart();

        if (isStopRequested()) return;

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        runningFor = new ElapsedTime();

        currentState = State.FIX_ANGLE;
        drive.followTrajectorySequenceAsync(fixAngle);

        while (opModeIsActive() && !isStopRequested()) {
            switchTrajs();

            if(runningFor.seconds() >= GO_PARK_AT)
            {
                currentState = State.LEAVE_EVERYTHING_AND_PARK;
            }

            if(rangeSensor.rawOptical() != emptyBox)
            {
                hasFreight = true;
            }
            else
            {
                hasFreight = false;
            }

            if(colorSensor.passedWearHouse())
            {
                Pose2d poseEstimate = drive.getPoseEstimate();
                drive.setPoseEstimate(new Pose2d(whiteLineX, poseEstimate.getY(), poseEstimate.getHeading()));
                whiteLineDetected=true;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();

            elevator.updateAuto();
            spinner.updateAuto();
//
            telemetry.addData("white line detected:",whiteLineDetected);
            telemetry.update();
//            // Read pose
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//            // Continually write pose to `PoseStorage`
//            PoseStorage.currentPose = poseEstimate;
        }
    }

    void changeState(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);
        }
    }

    void switchTrajs()
    {
        switch (currentState) {
            case FIX_ANGLE:
                changeState(State.OPEN_ELEVATOR, goToHubFirst);
                break;
            case OPEN_ELEVATOR:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY;
                }
                break;
            case WAIT_ELEVATOR_DELAY:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    switch (intakeType)
                    {
                        case STRAIGHT_PATH:
                            changeStateIntake(State.INTAKE, straightLineIntake);
                            break;
                        case FIFTEEN_DEGREE_PATH:
                            changeStateIntake(State.INTAKE, fifteenDegreeIntake);
                            break;
                        case THIRTY_DEGREE_PATH:
                            changeStateIntake(State.INTAKE, thirtyDegreeIntake);
                            break;
                    }
                }
                break;
            case INTAKE:
                if(hasFreight)
                {
                    drive.breakFollowing();
                }

                if (!drive.isBusy()) {
                    changeState(State.OPEN_ELEVATOR, goToHub);
                }
                break;
            case LEAVE_EVERYTHING_AND_PARK:
                if (drive.isBusy())
                {
                    drive.breakFollowing();

                    Pose2d currentPose = drive.getPoseEstimate();

                    TrajectorySequence parkNow = drive.trajectorySequenceBuilder(currentPose)
                            .lineToSplineHeading(poseCollect)
                            .build();

                    closeElevator();
                    intake.stop();

                    drive.followTrajectorySequenceAsync(parkNow);
                }
                else
                {
                    changeState(State.IDLE, park);
                }
                break;
            case IDLE:
                requestOpModeStop();
                break;
        }
    }

    void changeStateIntake(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);

            wentIntakeXTimes++;

            switch (wentIntakeXTimes)
            {
                case 1:
                    intakeType = intakePathType.STRAIGHT_PATH;
                    break;
                case 2:
                    intakeType = intakePathType.FIFTEEN_DEGREE_PATH;
                    break;
                case 3:
                    intakeType = intakePathType.THIRTY_DEGREE_PATH;
                    wentIntakeXTimes = 0;
                    break;
            }
        }
    }

    void openElevator()
    {
        cover.openCover();

        powerElevator = powerElevatorFast;
        elevator.setPower(powerElevator);

        elevator.updateAuto();
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        spinner.updateAuto();


        dip.holdFreight();

        hand.level3();

        elevator.updateAuto();
        spinner.updateAuto();
    }

    void closeElevator()
    {
        dip.releaseFreight();

        powerElevator = powerSlowElevator;
        elevator.setPower(powerElevator);

        elevator.updateAuto();
        spinner.updateAuto();

        hand.intake();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        elevator.updateAuto();
        spinner.updateAuto();

        cover.closeCover();
    }
}
