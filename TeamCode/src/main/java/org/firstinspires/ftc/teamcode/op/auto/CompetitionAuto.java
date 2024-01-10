package org.firstinspires.ftc.teamcode.op.auto;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.controller.PIDController;
import org.firstinspires.ftc.teamcode.model.Alliance;
import org.firstinspires.ftc.teamcode.model.RandomizationItem;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.teamcode.model.TurnDirection;
import org.firstinspires.ftc.teamcode.util.AngleUtils;
import org.firstinspires.ftc.teamcode.util.DebugLog;
import org.firstinspires.ftc.teamcode.vision.CustomPropLocationDetectorOpenCV;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "AutoComp", group = "Robot", preselectTeleOp = "Tele-Arcade")
public abstract class CompetitionAuto extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // neverest 20
    static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static double TURN_PID_P = 0.005;
    public static double TURN_PID_I = 0.01;
    public static double TURN_PID_D = 0.0005;
    public static int TICKS_PER_PIXEL_OUTTAKE = 90;
    public static double MAX_DRIVE_SPEED = 0.175;
    public static double MAX_TURN_SPEED = 0.1;
    public static int ARM_SCORING_POSITION = -354;
    public static int ARM_CAMERA_FORWARD_POSITION = -150;

    public static double SPIKE_INITIAL_FORWARD_DISTANCE = 16.0;
    public static double SPIKE_CENTER_FORWARD_DISTANCE = 9.0;
    public static double SPIKE_SIDE_FORWARD_DISTANCE = 4.0;
    public static double SHORT_PATH_CENTER_BACKDROP_FORWARD_DISTANCE = 24.0;  //From A4 / F4
    public static double SHORT_PATH_SIDE_BACKDROP_FORWARD_DISTANCE = 28.0;  //From A4 / F4
    public static double SHORT_PATH_INTERIOR_SIDE_BACKDROP_FORWARD_DISTANCE = 32.0;  //From A4 / F4
    public static double CENTER_STAGE_DISTANCE = 38.0;
    public static double CENTER_STAGE_TO_BACKDROP_DISTANCE = 72.0;  //From C2 / D2 to C5 / D5 respectively
    public static double TILE_DISTANCE = 24.0;
    public static double PIXEL_BACKUP_DISTANCE = 4.0; //to backup to avoid hitting the pixel when turning after dropping it on spike mark
    public static double BACKSTAGE_PARK_DISTANCE = 15.0;
    public static double SPIKE_CENTER_TURN_ANGLE = 5.0;
    public static double SPIKE_SIDE_TURN_ANGLE = 40.0;

    public static double BOARD_SIDE_ANGLE = 30.0; //TODO: test this on board

    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;
    private DcMotorEx arm1 = null;
    private DcMotorEx arm2 = null;
    private DcMotorEx intake1 = null;
    private DcMotorEx ext1 = null;
    private IMU imu = null;
    private Servo servo1;
    private VisionPortal visionPortal = null;
    private CustomPropLocationDetectorOpenCV customPropLocationDetector = null;
    private final RandomizationItem randomizationItem = RandomizationItem.PROP;

    private PIDController pidRotate;
    private ElapsedTime elapsedTime = null;

    private double globalAngle;
    private Orientation lastAngles = new Orientation();

    protected abstract Alliance getAlliance();

    protected abstract String getStartingPosition();

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();
        waitForStart();
        runAutoRoutine();
    }

    private void initBot() {
        left1 = hardwareMap.get(DcMotorEx.class, "L1");
        left2 = hardwareMap.get(DcMotorEx.class, "L2");
        right1 = hardwareMap.get(DcMotorEx.class, "R1");
        right2 = hardwareMap.get(DcMotorEx.class, "R2");

        arm1 = hardwareMap.get(DcMotorEx.class, "A1");
        arm2 = hardwareMap.get(DcMotorEx.class, "A2");
        intake1 = hardwareMap.get(DcMotorEx.class, "I1");
        ext1 = hardwareMap.get(DcMotorEx.class, "E1");

        servo1 = hardwareMap.get(Servo.class, "S1");

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ext1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left1.setDirection(DcMotorEx.Direction.REVERSE);
        left2.setDirection(DcMotorEx.Direction.FORWARD);
        right1.setDirection(DcMotorEx.Direction.REVERSE);
        right2.setDirection(DcMotorEx.Direction.FORWARD);
        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        intake1.setDirection(DcMotorEx.Direction.FORWARD);
        ext1.setDirection(DcMotorEx.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        ext1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ext1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pidRotate = new PIDController(TURN_PID_P, TURN_PID_I, TURN_PID_D);

        //setup FTC dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw(); //zero out our rotation angle
        elapsedTime = new ElapsedTime();

        customPropLocationDetector = new CustomPropLocationDetectorOpenCV(getAlliance(), telemetry);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        builder.setCameraResolution(new Size(1280, 720));
        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);
        // Set and enable the processors
        builder.addProcessors(customPropLocationDetector);
        visionPortal = builder.build();

        customPropLocationDetector.setVisionPortal(visionPortal);
    }

    private void scoreSpikePixel(TargetPosition targetPosition) {
        //---------------------------------------------------- PLACE PIXEL ON SPIKE ----------------------------------------------------
        DebugLog.log("Target scoring position is %s", targetPosition);
        //Note: approx. 21 inches from back of bot to edge of first pre-loaded pixel to drop - center tape mark at approx 47" from wall, 26" straight ahead

        DebugLog.log("Driving forward %.1f inches", SPIKE_INITIAL_FORWARD_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, SPIKE_INITIAL_FORWARD_DISTANCE, 3);

        sleep(250);  //wait little to ensure we're done moving

        //turn into pixel drop position
        if (getAlliance() == Alliance.BLUE) {
            if (targetPosition == TargetPosition.LEFT) {
                DebugLog.log("Turning left %.1f degrees", SPIKE_SIDE_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, TurnDirection.LEFT));
            } else if (targetPosition == TargetPosition.CENTER) {
                DebugLog.log("Turning right %.1f degrees", SPIKE_CENTER_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_CENTER_TURN_ANGLE, TurnDirection.RIGHT));
            } else if (targetPosition == TargetPosition.RIGHT) {
                DebugLog.log("Turning right %.1f degrees", SPIKE_SIDE_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, TurnDirection.RIGHT));
            }
        } else { //RED
            if (targetPosition == TargetPosition.LEFT) {
                DebugLog.log("Turning left %.1f degrees", SPIKE_SIDE_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, TurnDirection.LEFT));
            } else if (targetPosition == TargetPosition.CENTER) {
                DebugLog.log("Turning left %.1f degrees", SPIKE_CENTER_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_CENTER_TURN_ANGLE, TurnDirection.LEFT));
            } else if (targetPosition == TargetPosition.RIGHT) {
                DebugLog.log("Turning right %.1f degrees", SPIKE_SIDE_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, TurnDirection.RIGHT));
            }
        }

        sleep(250);

        //Drive forward so pixel is over/near the scoring spike location
        if (targetPosition == TargetPosition.CENTER) {
            DebugLog.log("Driving forward %.1f inches", SPIKE_CENTER_FORWARD_DISTANCE);
            driveStraight(MAX_DRIVE_SPEED, SPIKE_CENTER_FORWARD_DISTANCE, 4);
        } else {
            //left/right spike will be closer than center
            DebugLog.log("Driving forward %.1f inches", SPIKE_SIDE_FORWARD_DISTANCE);
            driveStraight(MAX_DRIVE_SPEED, SPIKE_SIDE_FORWARD_DISTANCE, 4);
        }

        //Drop the first pixel
        DebugLog.log("Dropping pixel!");
        outtake(1);

        //backup a bit to avoid driving over the dropped pixel when we turn towards the scoring backdrop / back stage area
        DebugLog.log("Driving backward %.1f inches", PIXEL_BACKUP_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, -PIXEL_BACKUP_DISTANCE, 2);

        sleep(250);
    }

    private void runAutoRoutine() {
        liftBlade("up");

        //---------------------------------------------------- VISION ----------------------------------------------------
        DebugLog.log("Starting at position %s, alliance %s", getStartingPosition(), getAlliance());
        TargetPosition targetPosition = getRandomizationTargetPosition();
        if (targetPosition == null) {
            //if we couldn't find a target using vision, assume it's on the right side,
            //since the camera is on the left side of the robot and cannot usually fully see the full right spike area
            targetPosition = TargetPosition.RIGHT;
        }

        scoreSpikePixel(targetPosition);

        //---------------------------------------------------- DRIVE TO BACKSTAGE LOGIC ----------------------------------------------------
        if (getStartingPosition().contains("2")) {
            //we are on the far side closest to the audience and have to drive the long way through the center stage door to get to the back stage area
            DebugLog.log("Starting in bottom half of field");

            //target is on the inside closest to truss, which is good since we can then drive straight ahead towards center of field without hitting dropped pixel
            if ((getAlliance() == Alliance.BLUE && targetPosition == TargetPosition.LEFT) || (getAlliance() == Alliance.RED && targetPosition == TargetPosition.RIGHT)) {
                lowerFieldDriveThroughSpikes();
            } else {
                lowerFieldDriveAroundSpikes(targetPosition);
            }
        } else {
            //we are in the top half and already past the trusses
            boolean takeShortPath = false;
            if (getAlliance() == Alliance.BLUE && (targetPosition == TargetPosition.CENTER || targetPosition == TargetPosition.RIGHT)) {
                takeShortPath = true;
            } else if (getAlliance() == Alliance.RED && (targetPosition == TargetPosition.CENTER || targetPosition == TargetPosition.LEFT)) {
                takeShortPath = true;
            }

            if (takeShortPath) {
                upperFieldDriveThroughSpikes(targetPosition);
            } else {
                upperFieldDriveAroundSpikes(targetPosition);
            }
        }

        //---------------------------------------------------- BACKDROP ----------------------------------------------------

        //we should now be in row 5 roughly in front of the backdrop - exact bot position will depend on initial randomization target
        if (targetPosition == TargetPosition.LEFT) {
            DebugLog.log("Turning left %.1f degrees", BOARD_SIDE_ANGLE);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(BOARD_SIDE_ANGLE, TurnDirection.LEFT));
        } else if (targetPosition == TargetPosition.RIGHT) {
            DebugLog.log("Turning right %.1f degrees", BOARD_SIDE_ANGLE);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(BOARD_SIDE_ANGLE, TurnDirection.RIGHT));
        }

        //TODO: use April Tags to orient ourselves to the correct L/C/R scoring slot?
        raiseArmToScoringPosition();
        driveStraight(MAX_DRIVE_SPEED,6,2);

        outtake(1);

        DebugLog.log("Backing up %.1f inches to avoid striking the backdrop", PIXEL_BACKUP_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, -PIXEL_BACKUP_DISTANCE, 5);

        returnArmToStartingPosition();

        //turn towards our final park position
        //TODO: test/adjust these angles on actual field - may require slight angle adjustments based on initial randomization target
        if (getAlliance() == Alliance.BLUE) {
            DebugLog.log("Turning left %.1f degrees", 90.0);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, TurnDirection.LEFT));
        } else {
            DebugLog.log("Turning left %.1f degrees", 90.0);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, TurnDirection.RIGHT));
        }

        //TODO: test/adjust this distance on actual field - may require slight distance adjustments based on initial randomization target
        DebugLog.log("Driving forward %.1f inches to final park position", BACKSTAGE_PARK_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, BACKSTAGE_PARK_DISTANCE, 5);

        liftBlade("down");
    }

    //starting position is in upper field, closest to the back stage - drive through spikes without pixel avoidance path
    private void upperFieldDriveThroughSpikes(TargetPosition targetPosition) {
        DebugLog.log("Taking short path to backstage area (no need to drive around dropped pixel)");

        if (getAlliance() == Alliance.BLUE) {
            if (targetPosition == TargetPosition.CENTER) {
                //we are 5 degrees off center (to the right) and 2 inches back from the spike mark
                DebugLog.log("Turning left %.1f degrees", 90 + SPIKE_CENTER_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90 + SPIKE_CENTER_TURN_ANGLE, TurnDirection.LEFT)); //turn left 95 degrees to get perpendicular to the scoring backdrop
            } else {
                //TargetPosition.RIGHT - we are 40 degrees off center (to the right) and 2 inches back from the spike mark
                DebugLog.log("Turning left %.1f degrees", 90 + SPIKE_SIDE_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90 + SPIKE_SIDE_TURN_ANGLE, TurnDirection.LEFT)); //turn left 130 degrees
            }
        } else { //RED
            if (targetPosition == TargetPosition.CENTER) {
                //we are 5 degrees off center (to the left) and 2 inches back from the spike mark
                DebugLog.log("Turning right %.1f degrees", 90 + SPIKE_CENTER_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90 + SPIKE_CENTER_TURN_ANGLE, TurnDirection.RIGHT)); //turn right 95 degrees to get perpendicular to the scoring backdrop
            } else {
                //TargetPosition.LEFT - we are 40 degrees off center (to the left) and 2 inches back from the spike mark
                DebugLog.log("Turning right %.1f degrees", 90 + SPIKE_SIDE_TURN_ANGLE);
                pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90 + SPIKE_SIDE_TURN_ANGLE, TurnDirection.RIGHT)); //turn right 130 degrees
            }
        }

        //drive forward to get into scoring position
        //TODO: verify these distances on actual field
        if (targetPosition == TargetPosition.CENTER) {
            DebugLog.log("Driving forward %.1f inches", SHORT_PATH_CENTER_BACKDROP_FORWARD_DISTANCE);
            driveStraight(MAX_DRIVE_SPEED, SHORT_PATH_CENTER_BACKDROP_FORWARD_DISTANCE, 5);
        } else {
            DebugLog.log("Driving forward %.1f inches", SHORT_PATH_SIDE_BACKDROP_FORWARD_DISTANCE);
            driveStraight(MAX_DRIVE_SPEED, SHORT_PATH_SIDE_BACKDROP_FORWARD_DISTANCE, 5);
        }
    }

    //starting position is in upper field, closest to the back stage - drive around spikes for pixel avoidance path
    private void upperFieldDriveAroundSpikes(TargetPosition targetPosition) {
        if (getAlliance() == Alliance.BLUE) {
            //TargetPosition.LEFT - we are 40 degrees off center (to the left) and 2 inches back from the spike mark
            DebugLog.log("Turning left %.1f degrees", 90 - SPIKE_SIDE_TURN_ANGLE);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, TurnDirection.LEFT));  //turn left 50 degrees to get get perpendicular to the scoring backdrop
        } else {
            //Red && TargetPosition.RIGHT - we are 40 degrees off center (to the right) and 2 inches back from the spike mark
            DebugLog.log("Turning right %.1f degrees", 90 - SPIKE_SIDE_TURN_ANGLE);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, TurnDirection.RIGHT));  //turn right 50 degrees to get get perpendicular to the scoring backdrop
        }

        //TODO: drive forward X inches to get into row 5, re-orient bot to get close enough to the scoring backdrop, rotate into correct scoring position,
        //  put arm into scoring position, outtake pixel, then drive into parking position in backstage

        //TODO: verify/test this distance on actual field
        DebugLog.log("Driving forward %.1f inches", SHORT_PATH_INTERIOR_SIDE_BACKDROP_FORWARD_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, SHORT_PATH_INTERIOR_SIDE_BACKDROP_FORWARD_DISTANCE, 5);
    }

    //starting position is in lower field, closest to the audience - drive through spikes without pixel avoidance path
    private void lowerFieldDriveThroughSpikes() {
        //backup to SPIKE_INITIAL_FORWARD_DISTANCE
        DebugLog.log("Driving backward %.1f inches", SPIKE_SIDE_FORWARD_DISTANCE - PIXEL_BACKUP_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, -(SPIKE_CENTER_FORWARD_DISTANCE - PIXEL_BACKUP_DISTANCE), 4);

        //revert angle
        TurnDirection turnDirection = (getAlliance() == Alliance.BLUE) ? TurnDirection.RIGHT : TurnDirection.LEFT;
        DebugLog.log("Turning %s %.1f degrees", turnDirection, SPIKE_SIDE_TURN_ANGLE);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, turnDirection));

        //drive forward into center stage area (col C for Blue, D for Red)
        //TODO: test/verify this distance on actual field
        DebugLog.log("Driving forward %.1f inches", CENTER_STAGE_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, CENTER_STAGE_DISTANCE, 6);

        //turn towards center stage door
        turnDirection = (getAlliance() == Alliance.BLUE) ? TurnDirection.LEFT : TurnDirection.RIGHT;
        DebugLog.log("Turning %s %.1f degrees", turnDirection, 90.0);

        //drive forward approx 3 full tiles to get into row 5
        DebugLog.log("Driving forward %.1f inches", (TILE_DISTANCE*3)+(TILE_DISTANCE/2));
        driveStraight(MAX_DRIVE_SPEED, (TILE_DISTANCE*3)+(TILE_DISTANCE/2), 5);

        DebugLog.log("Turning %s %.1f degrees", TurnDirection.LEFT, 90.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, turnDirection));

        DebugLog.log("Driving forward %.1f inches", TILE_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, TILE_DISTANCE, 2);

        turnDirection = (getAlliance() == Alliance.BLUE) ? TurnDirection.RIGHT : TurnDirection.LEFT;
        DebugLog.log("Turning %s %.1f degrees", TurnDirection.RIGHT, 90.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, turnDirection));
    }

    //starting position is in lower field, closest to the audience - drive around spikes for pixel avoidance path
    private void lowerFieldDriveAroundSpikes(TargetPosition targetPosition) {
        //we have to back up back into starting position to drive along the wall near the audience to reach center stage area
        TurnDirection turnDirection = (getAlliance() == Alliance.BLUE) ? TurnDirection.RIGHT : TurnDirection.LEFT;

        if ((getAlliance() == Alliance.BLUE && targetPosition == TargetPosition.CENTER) || (getAlliance() == Alliance.RED && targetPosition == TargetPosition.CENTER)) {
            //backup to SPIKE_INITIAL_FORWARD_DISTANCE
            DebugLog.log("Driving backward %.1f inches", SPIKE_SIDE_FORWARD_DISTANCE - PIXEL_BACKUP_DISTANCE);
            driveStraight(MAX_DRIVE_SPEED, -(SPIKE_CENTER_FORWARD_DISTANCE - PIXEL_BACKUP_DISTANCE), 4);
            //revert angle
            DebugLog.log("Turning %s %.1f degrees", turnDirection, SPIKE_CENTER_TURN_ANGLE);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_CENTER_TURN_ANGLE, turnDirection));
        } else {
            //backup to SPIKE_INITIAL_FORWARD_DISTANCE
            DebugLog.log("Driving backward %.1f inches", SPIKE_SIDE_FORWARD_DISTANCE - PIXEL_BACKUP_DISTANCE);
            driveStraight(MAX_DRIVE_SPEED, -(SPIKE_SIDE_FORWARD_DISTANCE - PIXEL_BACKUP_DISTANCE), 4);
            //revert angle
            DebugLog.log("Turning %s %.1f degrees", turnDirection, SPIKE_SIDE_TURN_ANGLE);
            pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(SPIKE_SIDE_TURN_ANGLE, turnDirection));
        }

        driveStraight(MAX_DRIVE_SPEED, -10, 4);
        //Turn towards audience wall
        //TODO: test/verify this angle on actual field - we need to make sure we don't drive into the wall or spike stacks along it
        DebugLog.log("Turning %s %.1f degrees", turnDirection, 45.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(45.0, turnDirection));
        //TODO: test/verify this distance on actual field - we need to make sure we don't drive into the wall or spike stacks along it
        DebugLog.log("Driving forward %.1f inches", 33.94112549695428);
        driveStraight(MAX_DRIVE_SPEED, 33.94112549695428, 4);

        //turn to drive straight along the audience wall
        turnDirection = (getAlliance() == Alliance.BLUE) ? TurnDirection.LEFT : TurnDirection.RIGHT;
        DebugLog.log("Turning %s %.1f degrees", turnDirection, 45.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(45.0, turnDirection));

        // drive forward approx 1 square
        // turn north towards center stage door
        DebugLog.log("Driving forward %.1f inches", TILE_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, TILE_DISTANCE, 4);

        DebugLog.log("Turning %s %.1f degrees", turnDirection, 90.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, turnDirection));

        DebugLog.log("Driving forward %.1f inches", ((TILE_DISTANCE * 4) + (TILE_DISTANCE/2)));
        driveStraight(MAX_DRIVE_SPEED, ((TILE_DISTANCE * 4) + (TILE_DISTANCE/2)), 6);

        DebugLog.log("Turning %s %.1f degrees", TurnDirection.LEFT, 90.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, turnDirection));

        DebugLog.log("Driving forward %.1f inches", TILE_DISTANCE);
        driveStraight(MAX_DRIVE_SPEED, TILE_DISTANCE, 2);

        DebugLog.log("Turning %s %.1f degrees", TurnDirection.RIGHT, 90.0);
        pidTurn(MAX_TURN_SPEED, AngleUtils.getRelativeTurnAngle(90.0, turnDirection));
    }

    private TargetPosition getRandomizationTargetPosition() {
        customPropLocationDetector.setDetectedPosition(null); //ensure we start with clean state
        double currentSeconds = elapsedTime.seconds();
        boolean pannedLeft = false;
        boolean pannedRight = false;
        double panLeftThreshold = currentSeconds + 1.5;
        double panRightThreshold = panLeftThreshold + 1.5;
        double timeThreshold = currentSeconds + 5.0;
        TargetPosition targetPosition = null;

        while (targetPosition == null && opModeIsActive() && elapsedTime.seconds() < timeThreshold) {
            if (elapsedTime.seconds() >= panLeftThreshold && !pannedLeft) {
                DebugLog.log("Panning left...");
                customPropLocationDetector.panLeft();
                pannedLeft = true;
            }
            if (elapsedTime.seconds() >= panRightThreshold && !pannedRight) {
                DebugLog.log("Panning right...");
                customPropLocationDetector.panRight();
                pannedRight = true;
            }
            targetPosition = customPropLocationDetector.getDetectedPosition();
            sleep(250);
        }

        customPropLocationDetector.resetPanTilt();
        visionPortal.setProcessorEnabled(customPropLocationDetector, false);

        if (targetPosition == null) {
            DebugLog.log("No target position found using vision");
        } else {
            DebugLog.log("Target position found using vision: %s", targetPosition);
        }

        /*
        //Save a screen capture of the camera view for debugging
        DateFormat dateFormat = new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss");
        Calendar cal = Calendar.getInstance();
        String timestamp = dateFormat.format(cal.getTime());
        customPropLocationDetector.getVisionPortal().saveNextFrameRaw(String.format(Locale.US, "SpikeCapture-%s", timestamp));
        sleep(300);
         */

        return targetPosition;
    }

    private void outtake(int numPixels) {
        int target = 0;
        //figure out how many pixels we want to drop
        if (numPixels == 1) {
            target = intake1.getCurrentPosition() + TICKS_PER_PIXEL_OUTTAKE;
        } else if (numPixels == 2) {
            target = intake1.getCurrentPosition() + (2 * TICKS_PER_PIXEL_OUTTAKE);
        } else {
            telemetry.addData("outtake() problem:", numPixels + " is not 1 or 2. try again stupid");
        }
        intake1.setTargetPosition(target);
        intake1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intake1.setPower(1.0);

        //run until we reach target position
        while (opModeIsActive() && intake1.isBusy()) {
            DebugLog.log("Outtaking - current position = %d, target position = %d", intake1.getCurrentPosition(), target);
        }

        intake1.setPower(0);
        intake1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void raiseArmToScoringPosition() {
        arm1.setTargetPosition(ARM_SCORING_POSITION);
        arm1.setPower(.5);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elapsedTime.reset();
        double raiseTimeoutSeconds = 5.0;

        while (opModeIsActive() && elapsedTime.seconds() < raiseTimeoutSeconds && arm1.isBusy()) {
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setPower(.5);
            DebugLog.log("Raising arm to scoring position %d, current pos %d", ARM_SCORING_POSITION, arm1.getCurrentPosition());
        }

        arm1.setPower(0);
        arm2.setPower(0);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void raiseArmToCameraForwardPosition() {
        arm1.setTargetPosition(ARM_CAMERA_FORWARD_POSITION);
        arm1.setPower(.5);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elapsedTime.reset();
        double raiseTimeoutSeconds = 5.0;

        while (opModeIsActive() && elapsedTime.seconds() < raiseTimeoutSeconds && arm1.isBusy()) {
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setPower(.5);
            DebugLog.log("Raising arm to scoring position %d, current pos %d", ARM_CAMERA_FORWARD_POSITION, arm1.getCurrentPosition());
        }

        arm1.setPower(0);
        arm2.setPower(0);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void returnArmToStartingPosition() {
        int adjustedPosition = (int) Math.round(ARM_SCORING_POSITION * 0.95); //take a little speed and distance off, so we don't slam the arm into the ground
        arm1.setTargetPosition(-adjustedPosition);
        arm1.setPower(.25);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elapsedTime.reset();
        double raiseTimeoutSeconds = 2.0;

        while (opModeIsActive() && elapsedTime.seconds() < raiseTimeoutSeconds && arm1.isBusy()) {
            arm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm2.setPower(-.25);
            DebugLog.log("Returning arm to initial position %d, current pos %d", -adjustedPosition, arm1.getCurrentPosition());
        }

        arm1.setPower(0);
        arm2.setPower(0);
        arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void pidTurn(double maxTurnSpeed, double degrees) {
        resetAngle();
        double power = maxTurnSpeed;

        pidRotate.reset();
        pidRotate.setPID(TURN_PID_P, TURN_PID_I, TURN_PID_D);
        pidRotate.setSetPoint(degrees);
        pidRotate.setTolerance(1); //get within 1 degree of desired angle

        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
                DebugLog.log("Turn - get off zero!  current heading = %.1f, calculated power = %.2f", getAngle(), power);
                left1.setPower(power);
                left2.setPower(power);
                right1.setPower(-power);
                right2.setPower(-power);
                sleep(100);
            }

            do {
                double currentDegrees = getAngle();
                power = pidRotate.calculate(currentDegrees); // power will be negative on right turn.
                DebugLog.log("Turning right!  current heading = %.1f, calculated power = %.2f", currentDegrees, power);
                left1.setPower(-power);
                left2.setPower(-power);
                right1.setPower(power);
                right2.setPower(power);
            } while (opModeIsActive() && !pidRotate.atSetPoint());
        } else {
            // left turn.
            do {
                double currentDegrees = getAngle();
                power = pidRotate.calculate(currentDegrees); // power will be positive on left turn.
                DebugLog.log("Turning left!  current heading = %.1f, calculated power = %.2f", currentDegrees, power);
                left1.setPower(-power);
                left2.setPower(-power);
                right1.setPower(power);
                right2.setPower(power);
            } while (opModeIsActive() && !pidRotate.atSetPoint());
        }

        // turn the motors off.
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);

        double rotation = getAngle();

        // wait for rotation to stop.
        sleep(250);

        // reset angle tracking on new heading.
        resetAngle();
        DebugLog.log("completing pidTurn - rotation = %.1f, lastAngles = %s", rotation, lastAngles);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void driveStraight(double speed, double distanceInInches, double timeoutSeconds) {
        dumbDrive(speed, distanceInInches, distanceInInches, timeoutSeconds);
    }

    private void dumbDrive(double speed,
                           double leftInches, double rightInches,
                           double timeoutS) {

        int newLeft1Target;
        int newLeft2Target;
        int newRight1Target;
        int newRight2Target;

        DebugLog.log("Dumbdrive start - L1 Pos %7d, R1 Pos %7d", left1.getCurrentPosition(), right1.getCurrentPosition());

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeft1Target = left1.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeft2Target = left2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRight1Target = right1.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRight2Target = right2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            left1.setTargetPosition(newLeft1Target);
            left2.setTargetPosition(newLeft2Target);
            right1.setTargetPosition(newRight1Target);
            right2.setTargetPosition(newRight2Target);

            // Turn On RUN_TO_POSITION
            left1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            elapsedTime.reset();
            //TODO: switch this to use PID control so we don't overshoot our target?
            left1.setPower(Math.abs(speed));
            left2.setPower(Math.abs(speed));
            right1.setPower(Math.abs(speed));
            right2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (elapsedTime.seconds() < timeoutS) &&
                    (left1.isBusy() && right1.isBusy())) {
                // Display it for the driver.
                telemetry.setMsTransmissionInterval(20);
                telemetry.addData("Running to", " %7d :%7d", newLeft1Target, newRight1Target);
                telemetry.addData("Currently at", " at %7d :%7d", left1.getCurrentPosition(), right1.getCurrentPosition());
                //sleep(20);
                telemetry.update();
            }

            //TODO: update current position

            telemetry.setMsTransmissionInterval(250);  //restore original default telemetry transmission interval
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);

            left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }
    private void liftBlade(String TRGT){
        if (TRGT.equals("up"))
            servo1.setPosition(0);
        else if (TRGT.equals("down")) {
            servo1.setPosition(.7);
        }
    }
}
