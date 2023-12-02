package org.firstinspires.ftc.teamcode.op.auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.model.Alliance;
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

    /* Declare OpMode members. */
    private DcMotorEx left1 = null;
    private DcMotorEx left2 = null;
    private DcMotorEx right1 = null;
    private DcMotorEx right2 = null;

    //basic typical opmode stuff
    private final ElapsedTime runtime = new ElapsedTime();
    private final Alliance alliance = Alliance.BLUE;

    //math to make convert motor rotations to wheel distance
    static final double COUNTS_PER_MOTOR_REV = 537.6;    // neverest 20
    static final double DRIVE_GEAR_REDUCTION = (15.0 / 20.0);     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //speeds
    static final double DRIVE_SPEED = 0.1;
    static final double TURN_SPEED = 0.5;
    final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

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
        //set default motor directions
        left1.setDirection(DcMotorEx.Direction.FORWARD);
        left2.setDirection(DcMotorEx.Direction.REVERSE);

        right1.setDirection(DcMotorEx.Direction.FORWARD);
        right2.setDirection(DcMotorEx.Direction.REVERSE);

        left1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        left1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                left1.getCurrentPosition(),
                left2.getCurrentPosition(),
                right1.getCurrentPosition(),
                right2.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //------------------------------------------------------------------------------------------
        //                               ! START ROUTINE HERE !
        //------------------------------------------------------------------------------------------

        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout

        visionPortal.setActiveCamera(webcam2);

        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(RTDP, true);

        //     --PLACEHOLDER--
        // place pixel on the spike using RTDP

        visionPortal.setProcessorEnabled(RTDP, false);
        visionPortal.setProcessorEnabled(aprilTag, true);



        while (opModeIsActive()) {

            //TELEMETRY
            telemetryAprilTag();
            telemetry.update();

            //telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);  // pause to display final telemetry message.
        }
    }
    public void encoderDrive(double speed,
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
            left1.setVelocity(Math.abs(speed));
            left2.setVelocity(Math.abs(speed));
            right1.setVelocity(Math.abs(speed));
            right2.setVelocity(Math.abs(speed));

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

    public void initDoubleVision() {
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
        RTDP = new RandomizationTargetDeterminationProcessor(alliance, telemetry);
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
        left1.setVelocity(leftPower);
        left2.setVelocity(leftPower);
        right1.setVelocity(rightPower);
        right2.setVelocity(rightPower);
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
    private  void runApril(){
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
                /*TODO: do math to figure out our position
                 * use RobotAutoDriveByGyro_Linear as an example to write some path planning code (maybe)
                 */


                positionX = (chosenTag.rawPose.x + chosenTag.ftcPose.x);
                positionY = (chosenTag.rawPose.y + chosenTag.ftcPose.y);  //I made this up I have no idea if it works


            }
        }
    }
    private  void runApril(int iWantThisOne){
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
                double headingError = chosenTag.ftcPose.bearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.  Clip it to the maximum
                double drive = Range.scale(Range.clip(rangeError * SPEED_GAIN, -DRIVE_SPEED, DRIVE_SPEED), 1, -1, 160, -160);
                double turn = Range.scale(Range.clip(headingError * TURN_GAIN, -TURN_SPEED, TURN_SPEED), 1, -1, 160, -160);

                telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn);
            }
        }
    }
}