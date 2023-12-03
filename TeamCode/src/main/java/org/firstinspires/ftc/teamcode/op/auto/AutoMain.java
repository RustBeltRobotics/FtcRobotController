package org.firstinspires.ftc.teamcode.op.auto;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.model.Alliance;
import org.firstinspires.ftc.teamcode.model.RandomizationItem;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.teamcode.opencv.RandomizationTargetDeterminationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Auto-comp", group="Robot")
//@Disabled
public class AutoMain extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    //                                       CONFIG VARS
    //----------------------------------------------------------------------------------------------

    //starting positions as defined in the back of game-manual-part-2-traditional
    private String startingPositionLetter = "A";
    private int startingPositionNumber = 2;
    //tuning variables for movement
    static final double DRIVE_SPEED = 5;
    static double TURN_SPEED = 0.5;
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.0005 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    private double  targetHeading = 0;
    private double headingError = 0.0;
    static double HEADING_THRESHOLD = 5.0 ; // How close must the heading get to the target before moving to next step.
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
    private final Alliance alliance = Alliance.BLUE;
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
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    // irobot
    private double positionX;
    private double positionY;
    private double robotBearing;
    @Override
    public void runOpMode() {
        // Initialize the Apriltag Detection process
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

        //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                left1.getCurrentPosition(),
                left2.getCurrentPosition(),
                right1.getCurrentPosition(),
                right2.getCurrentPosition());
        telemetry.addData("intake pos:", intake1.getCurrentPosition());
        telemetry.update();

        initPose(startingPositionLetter, startingPositionNumber);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        //------------------------------------------------------------------------------------------
        //                               ! START ROUTINE HERE !
        //------------------------------------------------------------------------------------------

        dumbDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        visionPortal.setActiveCamera(webcam2);

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(RTDP, true);

        //RTDP.setDetectedPosition(TargetPosition.LEFT);
        while (RTDP.getDetectedPosition() == null){
           telemetry.addData("Status", "you can hide but you cant run ( where is the tape :( )");
           telemetry.update();
        }

        //TODO: move intake to the spike using RTDP

        outtake(1);
        updatePosApril(); //update position with big april tags

        visionPortal.setProcessorEnabled(RTDP, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        int targetAprilId = scoringTag(RTDP.getDetectedPosition());

        if (alliance == Alliance.BLUE) {
            smartDrive(-24.0, 30.5);
            smartDrive(24, 30.5);

            goToTag(targetAprilId);
            turnToHeading(1, 0);

            moveArm("scoring");
            outtake(1);

        } else { //fun fact the alliance will be red
            //red code
            telemetry.addData("Status:","PLEASE I WANT TO BE BLUE TURN IT BACK TURN IT BACK");
        }

        while (opModeIsActive()) {
            //TELEMETRY
            telemetry.addData("Heading err:", headingError);
            telemetry.addData("Heading:", getHeading());
            telemetry.addData("arm reading:", intake1.getCurrentPosition());
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
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

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left1.isBusy() && right1.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeft1Target, newRight1Target);
                telemetry.addData("Currently at", " at %7d :%7d",
                        left1.getCurrentPosition(), right1.getCurrentPosition());

                telemetry.addData("Heading err", headingError);
                telemetry.addData("Heading", getHeading());
                telemetry.update();

            }

            // Stop all motion;
            left1.setPower(0);
            left2.setPower(0);
            right1.setPower(0);
            right2.setPower(0);

            // Turn off RUN_TO_POSITION
            left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    // *NEW* moves the arm into one of two super convenient positions !
    private void moveArm(String thingToDo){
        int scoringPos = 0;  //TODO: add real values for these
        int intakePos = 0;

        // Turn On RUN_TO_POSITION
        arm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (thingToDo == "scoring"){
            arm1.setTargetPosition(scoringPos);
            arm2.setTargetPosition(scoringPos);
        } else if (thingToDo == "intake") {
            arm1.setTargetPosition(intakePos);
            arm2.setTargetPosition(intakePos);
        } else { telemetry.addData("moveArm() err:", "invalid value given"); telemetry.update(); }
    }
    private void outtake(int numPixels){
        int target = 0;
        int magicNumber = 0; //magicNumber is the distance the intake encoder has to move for a pixel to be outtaken
        //figure out how many pixels we want to drop
        if (numPixels == 1) {
            target = arm1.getCurrentPosition() + magicNumber; //TODO: add real values for these
        } else if (numPixels == 2) {
            target = arm1.getCurrentPosition() + (2 * magicNumber);
        } else { telemetry.addData("outtake() problem:", numPixels + " is not 1 or 2. try again stupid"); }

        arm1.setTargetPosition(target);
        arm2.setTargetPosition(target);
    }
    private void initDoubleVision() {
        // AprilTag Configuration
        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(678.154, 678.17, 318.135, 228.374)
                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // TFOD Configuration
        tfod = new TfodProcessor.Builder()
                .build();
        //RTDP Initialization
        RTDP = new RandomizationTargetDeterminationProcessor(alliance, randomizationItem, telemetry);
        // -----------------------------------------------------------------------------------------
        // Camera Configuration

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(tfod, aprilTag, RTDP)
                .build();
    }// end initDoubleVision()

    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }
    public void moveRobot(double x, double yaw) {
        // Calculate left and right wheel powers.
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >160.0) {
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
    private  void updatePosApril(){
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        chosenTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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
    private  void goToTag(int iWantThisOne){
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
                //take 9 inches off of the raw position to adjust for robot length and
                //TODO: ADJUST THE -9 TO ACCOUNT FOR THE ARM LENGTH
                smartDrive(chosenTag.rawPose.x-9,chosenTag.rawPose.y);
            }
        }
    }
    public void initPose(String startingPositionY, int startingPositionX){
        double[] arrayOfPossibilities = {-30.0,-18.0,-6.0,6.0,18.0,30.0};
        double A = 28.0;
        double F = -28.0;
        double YstartPos = 0;

        switch (startingPositionY.substring(0,1)){
            case "A":
                YstartPos = 28.0;
                break;
            case "F":
                YstartPos = -28.0;
                break;
        }

        positionY = YstartPos;
        positionX = arrayOfPossibilities[startingPositionX - 1];

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();



        //TODO: ADD ROBOT CENTER OFFSET FROM CONTROLLER IMU
    }
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = (targetHeading - getHeading());

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

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
            TURN_SPEED = Range.clip(TURN_SPEED, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, TURN_SPEED);
            //update our position to account for slippage
            updatePosApril();
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        if (alliance == Alliance.BLUE) {
            return -(orientation.getYaw(AngleUnit.DEGREES) + 270);
        } else {
            return -(orientation.getYaw(AngleUnit.DEGREES) + 90);
        }
    }
    private void smartDrive(double X, double Y) {
        double triX = positionX - X;
        double triY = positionY - Y;

        double headingIWant = Math.atan2(triY, triX);
        turnToHeading(1, headingIWant);

        double range = Math.sqrt((Math.pow(triX,2) + (Math.pow(triY, 2))));
        dumbDrive(1,range,range,30);
    }
    private int scoringTag(TargetPosition targetPos) {
         switch (targetPos){
            case LEFT:
                return alliance == Alliance.BLUE ? 1 : 4;
            case CENTER:
                return alliance == Alliance.BLUE ? 2 : 5;
            case RIGHT:
                return alliance == Alliance.BLUE ? 3 : 6;
        }
     return -1;
    }
}