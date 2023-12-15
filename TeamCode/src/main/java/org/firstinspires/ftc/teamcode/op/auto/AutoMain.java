package org.firstinspires.ftc.teamcode.op.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.model.Alliance;
import org.firstinspires.ftc.teamcode.model.RandomizationItem;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.teamcode.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.vision.RandomizationTargetDeterminationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Config
@Autonomous(name="Auto-comp", group="Robot")
//@Disabled
public class AutoMain extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    //                                       CONFIG VARS
    //----------------------------------------------------------------------------------------------

    //starting positions as defined in the back of game-manual-part-2-traditional
    public static String startingPositionLetter = "F";
    public static int startingPositionNumber = 2;

    //tuning variables for movement
    public static double DRIVE_SPEED = .25;
    public static double TURN_SPEED = 0.5;
    public static double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double TURN_GAIN = 0.005 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public static double HEADING_THRESHOLD = 5.0 ; // How close must the heading get to the target before moving to next step.
    public static int hope = 2;
    //----------------------------------------------------------------------------------------------

    /* Declare OpMode members. */
    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;
    private DcMotorEx arm1 = null;
    private DcMotorEx arm2 = null;
    private DcMotorEx intake1 = null;

    private IMU imu = null;

    //basic typical opmode stuff
    private final ElapsedTime runtime = new ElapsedTime();
    private final Alliance alliance = Alliance.RED;
    private final RandomizationItem randomizationItem = RandomizationItem.PIXEL;

    //math to make convert motor rotations to wheel distance
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // neverest 20
    static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);


    //super intelligent vision stuff
    private RandomizationTargetDeterminationProcessor RTDP;
    private static final int DESIRED_TAG_ID = -1;
    public AprilTagDetection chosenTag = null;
    public WebcamName webcam1, webcam2;

    //For sending OpenCV image frames to FTC Dashboard
    private CameraStreamProcessor cameraStreamProcessor;
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    // irobot
    private double positionX;
    private double positionY;
    private double robotBearing;
    private double  targetHeading = 0;
    private double headingError = 0.0;


    @Override
    public void runOpMode() {
        targetHeading = 0;
        headingError = 0.0;

        // Initialize the vision processors for webcams
        initDoubleVision();

        // Initialize the drive system variables.
        left1 = hardwareMap.get(DcMotorEx.class, "L1");
        left2 = hardwareMap.get(DcMotorEx.class, "L2");

        right1 = hardwareMap.get(DcMotorEx.class, "R1");
        right2 = hardwareMap.get(DcMotorEx.class, "R2");

        arm1 = hardwareMap.get(DcMotorEx.class, "A1");
        arm2 = hardwareMap.get(DcMotorEx.class, "A2");

        intake1 = hardwareMap.get(DcMotorEx.class, "I1");

        //set default motor directions
        left1.setDirection(DcMotorEx.Direction.REVERSE);
        left2.setDirection(DcMotorEx.Direction.FORWARD);
        right1.setDirection(DcMotorEx.Direction.REVERSE);
        right2.setDirection(DcMotorEx.Direction.FORWARD);
        arm1.setDirection(DcMotorEx.Direction.FORWARD);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        intake1.setDirection(DcMotorEx.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //setup FTC dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                left1.getCurrentPosition(),
                left2.getCurrentPosition(),
                right1.getCurrentPosition(),
                right2.getCurrentPosition());
        telemetry.addData("intake pos:", intake1.getCurrentPosition());
        telemetry.update();

        initPose(startingPositionLetter, startingPositionNumber);

        visionPortal.setActiveCamera(webcam2);

        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("alliance", alliance);
        telemetry.update();

        //test robotlog
        RobotLog.dd("status", "hi computer");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        //------------------------------------------------------------------------------------------
        //                               ! START ROUTINE HERE !
        //------------------------------------------------------------------------------------------

        turnToHeading(TURN_SPEED, 20);


        smartDrive(-24,-24);
        smartDrive(24,-24);
        smartDrive(24,24);
        smartDrive(-24,24);



        visionPortal.setActiveCamera(webcam1);
        visionPortal.setProcessorEnabled(aprilTag, true);
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(RTDP, true);
        FtcDashboard.getInstance().startCameraStream(RTDP, 0);

        RTDP.setDetectedPosition(null);
        while (RTDP.getDetectedPosition() == null) {
            RobotLog.dd("status","looking for target spike mark");
            telemetry.addData("Status", "where is the tape :(");
            telemetry.update();
        }
        FtcDashboard.getInstance().stopCameraStream();
        visionPortal.setProcessorEnabled(RTDP, false);
        visionPortal.setProcessorEnabled(cameraStreamProcessor, true);
        FtcDashboard.getInstance().startCameraStream(cameraStreamProcessor, 0);

//        visionPortal.setActiveCamera(webcam1);
//        updatePosApril();
//        visionPortal.setActiveCamera(webcam2);

        RobotLog.dd("rtdp",RTDP.getDetectedPosition().toString());
        RobotLog.dd("status","i think i found the tape");
        //position the intake over the correct spike mark
        moveToSpike(RTDP.getDetectedPosition());
        //place 1 pixel
        outtake(1);
        //back up to avoid moving the pixel by accident
        dumbDrive(1, -1, -1, 1);
        //update position with big april tags
        updatePosApril();

        visionPortal.setProcessorEnabled(RTDP, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        int targetAprilId = scoringTag(RTDP.getDetectedPosition());
        //route to follow for blue side
        if (alliance == Alliance.BLUE) {
            smartDrive(-24.0, 30.5);
            smartDrive(24, 30.5);

            goToTag(targetAprilId);
            turnToHeading(1, 0);

            moveArm("scoring");
            outtake(1);

        } else if (alliance == Alliance.RED){ //route to follow on the red side
            smartDrive(-24.0, -30.5);
            smartDrive(24, -30.5);

            goToTag(targetAprilId);
            turnToHeading(1, 0);

            moveArm("scoring");
            outtake(1);
        }
    }

    public void dumbDrive(double speed,
                          double leftInches, double rightInches,
                          double timeoutS) {

        int newLeft1Target;
        int newLeft2Target;
        int newRight1Target;
        int newRight2Target;

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
            runtime.reset();
            left1.setPower(Math.abs(speed));
            left2.setPower(Math.abs(speed));
            right1.setPower(Math.abs(speed));
            right2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left1.isBusy() && right1.isBusy())) {
                updatePosApril();
                // Display it for the driver.
                telemetry.setMsTransmissionInterval(20);
                telemetry.addData("Running to", " %7d :%7d", newLeft1Target, newRight1Target);
                telemetry.addData("Currently at", " at %7d :%7d", left1.getCurrentPosition(), right1.getCurrentPosition());
                telemetry.addData("X", positionX);
                telemetry.addData("Y", positionY);
                telemetry.addData("Heading err", headingError);
                telemetry.addData("Heading", getHeading());
                //sleep(20);
                telemetry.update();
            }

            telemetry.setMsTransmissionInterval(250);  //restore original default telemetry transmission interval
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);

            left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    // *NEW* moves the arm into one of two super convenient positions !
    private void moveArm(String thingToDo) {
        int scoringPos = 0;  //TODO: add real values for these
        int intakePos = 0;

        // Turn On RUN_TO_POSITION
        arm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (thingToDo == "scoring") {
            arm1.setTargetPosition(scoringPos);
            arm2.setTargetPosition(scoringPos);
        } else if (thingToDo == "intake") {
            arm1.setTargetPosition(intakePos);
            arm2.setTargetPosition(intakePos);
        } else {
            telemetry.addData("moveArm() err:", "invalid value given");
            telemetry.update();
        }
    }
    //drop pixels with input for 1 or 2 pixels to be dropped
    private void outtake(int numPixels) {
        int target = 0;
        int magicNumber = 300; //magicNumber is the distance the intake encoder has to move for a pixel to be outtaken
        //figure out how many pixels we want to drop
        if (numPixels == 1) {
            target = intake1.getCurrentPosition() + magicNumber; //TODO: add real values for these
        } else if (numPixels == 2) {
            target = intake1.getCurrentPosition() + (2 * magicNumber);
        } else {
            telemetry.addData("outtake() problem:", numPixels + " is not 1 or 2. try again stupid");
        }
        intake1.setTargetPosition(target);
    }

    //code for initializing 2 webcams in a virtual camera which we can switch between 2 webcams.
    //also add the vision processors
    private void initDoubleVision() {
        cameraStreamProcessor = new CameraStreamProcessor();
        // AprilTag Configuration
        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(678.154, 678.17, 318.135, 228.374)
                .build();
        aprilTag.setDecimation(2);

        // TFOD Configuration
        tfod = new TfodProcessor.Builder()
                .build();
        //RTDP Initialization
        RTDP = new RandomizationTargetDeterminationProcessor(alliance, randomizationItem, telemetry);

        // Camera Configuration
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(tfod, aprilTag, RTDP, cameraStreamProcessor)
                .build();

    }// end initDoubleVision()

    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower = x - yaw;
        double rightPower = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 160.0) {
            leftPower /= max;
            rightPower /= max;
        }
        // Send powers to the wheels.
        left1.setPower(leftPower);
        left2.setPower(leftPower);
        right1.setPower(rightPower);
        right2.setPower(rightPower);
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    //this is a No-Argument constructor which updates the consciousness of the self aware super-robot in 3D-space
    private void updatePosApril() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        chosenTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            RobotLog.dd("April", String.valueOf(detection.id));
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                //tags 7&10 are the big AprilTags for the playing field wall
                if ((detection.id == 7 || detection.id == 10)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    chosenTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
            if (targetFound) {
                //TODO: check this to make sure it actually works
                positionX = (chosenTag.rawPose.x + chosenTag.ftcPose.x);
                positionY = (chosenTag.rawPose.y + chosenTag.ftcPose.y);
                RobotLog.dd("completed","position update from april");
            }
        }
    }
    private void updatePosApril(double X, double Y) {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        chosenTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if (currentDetections.size() == 0) {
            positionX = X;
            positionY = Y;
        } else {
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    //tags 7&10 are the big AprilTags for the playing field wall
                    if ((DESIRED_TAG_ID < 0) || (detection.id == 7 || detection.id == 10)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        chosenTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
                if (targetFound) {
                    //TODO: check this to make sure it actually works
                    positionX = (chosenTag.rawPose.x + chosenTag.ftcPose.x);
                    positionY = (chosenTag.rawPose.y + chosenTag.ftcPose.y);
                }
            }
        }
    }
    //makes the robot drive to an aprilTag.
    private void goToTag(int iWantThisOne) {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        chosenTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                //tags 7-10 are the AprilTags for the playing field wall
                if ((iWantThisOne < 0) || detection.id == iWantThisOne) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    chosenTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
            if (targetFound) {

                // Determine heading and range error so we can use them to control the robot automatically.
                double rangeError = (chosenTag.ftcPose.range - 12); //second number is desired distance from tag
                double headingError = -chosenTag.ftcPose.bearing;
                telemetry.addData("Heading err:", headingError);
                telemetry.addData("Heading:", getHeading());
                //take 9 inches off of the raw position to adjust for robot length and
                //TODO: ADJUST THE -9 TO ACCOUNT FOR THE ARM LENGTH
                smartDrive(chosenTag.rawPose.x - 12, chosenTag.rawPose.y);
            }
        }
    }
    //set correct position at the beginning of the match
    public void initPose(String startingPositionY, int startingPositionX) {
        double[] arrayOfPossibilities = {-60.0, -36.0, -12.0, 12.0, 36.0, 60.0};
        double A = 28.0;
        double F = -28.0;
        double YstartPos = 0;

        switch (startingPositionY.substring(0, 1)) {
            case "A":
                YstartPos = 63.5;
                break;
            case "F":
                YstartPos = -63.5;
                break;
        }

        positionY = YstartPos;
        positionX = arrayOfPossibilities[startingPositionX - 1];

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();
        telemetry.addData("Heading err:", headingError);
        telemetry.addData("Heading:", getHeading());

        // Normalize the error to be within +/- 180 degrees
        headingError = AngleUnit.normalizeDegrees(headingError);

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            TURN_SPEED = getSteeringCorrection(heading, TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            TURN_SPEED = Range.clip(TURN_SPEED, maxTurnSpeed, -maxTurnSpeed);

            // Pivot in place by applying the turning correction
            RobotLog.dd("heading:", String.valueOf(getHeading()));
            RobotLog.dd("heading err:", String.valueOf(headingError));
            moveRobot(0, TURN_SPEED);
            //update our position to account for slippage
            updatePosApril();
        }
        // Stop all motion;
        RobotLog.dd("completed:", "turnToHeading");
        moveRobot(0, 0);
    }

    //return the current rotational angle of the robot in the XY plane
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
//        if (alliance == Alliance.BLUE) {
//            return -(orientation.getYaw(AngleUnit.DEGREES) + 180);
//        } else {
//            return -(orientation.getYaw(AngleUnit.DEGREES));
//        }
    }

    //drive to a point on the field using X and Y coordinates in (units inches)
    private void smartDrive(double Y, double X) {
        double triX = positionX - X; //SWAPPED
        double triY = positionY - Y;
        RobotLog.dd("triangle X:", String.valueOf(triX));
        RobotLog.dd("triangle Y:", String.valueOf(triY));

        double headingIWant = -Math.toDegrees(Math.atan(triX/triY));
        RobotLog.dd("triangle angle",String.valueOf(headingIWant));
        turnToHeading(1, headingIWant);

        double range = Math.sqrt((Math.pow(triX, 2) + (Math.pow(triY, 2))));
        dumbDrive(DRIVE_SPEED, range, range, 30);
        updatePosApril(X,Y);
        //account for getting in an accident
        //updatePosApril();
//        if (hope == 0){
//            getDesperate(X,Y);
//        } else if ((Math.abs(positionX - X) > .5) || (Math.abs(positionY - Y) > .5)) {
//            hope--;
//            smartDrive(X,Y);
//        }
    }
    private void getDesperate(double X, double Y) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        while((currentDetections.size() == 0)) {
            left1.setPower(.5);
            left2.setPower(.5);
            right1.setPower(-.5);
            right2.setPower(-.5);
            currentDetections = aprilTag.getDetections();
        }
        left1.setPower(0);
        left2.setPower(0);
        hope = 3;
        smartDrive(X,Y);
        RobotLog.dd("completed", "getDesparate()");
    }
    //return the id of the aprilTag which corresponds to the spike mark the randomized pixel was on.
    private int scoringTag(TargetPosition targetPos) {
        switch (targetPos) {
            case LEFT:
                return alliance == Alliance.BLUE ? 1 : 4;
            case CENTER:
                return alliance == Alliance.BLUE ? 2 : 5;
            case RIGHT:
                return alliance == Alliance.BLUE ? 3 : 6;
        }
        return -1;
    }
    private void moveToSpike(TargetPosition detectedPosition) {
        /*
         *  because the starting position of the robot reletive to the spike marks should be the same
         *  weather we are on the top or bottom of the field, we do not account for starting position.
         *  We do have to account for the heading being different depending on the alliance.
         */

        if (alliance == Alliance.BLUE) {
            switch (detectedPosition) {
                //TODO: "fill these in later"
                case LEFT:
                    turnToHeading(1, 0);
                    dumbDrive(0.5, .2, .2, 5);
                case CENTER:
                    turnToHeading(1, 0);
                    dumbDrive(0.5, .2, .2, 5);
                case RIGHT:
                    turnToHeading(1, 0);
                    dumbDrive(0.5, .2, .2, 5);
            }
        } else if (alliance == Alliance.RED) {
            switch (detectedPosition) {
                //TODO: "fill these in later"
                case LEFT:
                    turnToHeading(1, 0);
                    dumbDrive(0.5, .2, .2, 5);
                case CENTER:
                    turnToHeading(1, 0);
                    dumbDrive(0.5, .2, .2, 5);
                case RIGHT:
                    turnToHeading(1, 0);
                    dumbDrive(0.5, .2, .2, 5);
            }
        }
    }

}