package org.firstinspires.ftc.teamcode.op.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.model.Alliance;
import org.firstinspires.ftc.teamcode.model.RandomizationItem;
import org.firstinspires.ftc.teamcode.util.DebugLog;
import org.firstinspires.ftc.teamcode.vision.CustomPropLocationDetector;
import org.firstinspires.ftc.teamcode.vision.RandomizationTargetDeterminationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Camera-Settings", group = "Robot")
public class CameraSettingsOp extends LinearOpMode {
    private VisionPortal visionPortal = null;        // Used to manage the video source.
    private WebcamName webcam2;
    private int myExposure;
    private int minExposure;
    private int maxExposure;
    private int myGain;
    private int minGain;
    private int maxGain;
    private int myPan;
    private int myTilt;
    private int myZoom;
    private int minPan;
    private int maxPan;
    private int minTilt;
    private int maxTilt;
    private int minZoom;
    private int maxZoom;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;
    boolean thisPanUp = false;
    boolean thisPanDn = false;
    boolean thisTiltUp = false;
    boolean thisTiltDn = false;
    boolean thisZoomUp = false;
    boolean thisZoomDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;
    boolean lastPanUp = false;
    boolean lastPanDn = false;
    boolean lastTiltUp = false;
    boolean lastTiltDn = false;
    boolean lastZoomUp = false;
    boolean lastZoomDn = false;

    @Override
    public void runOpMode() {
        DebugLog.log("RunOpMode - before initOp()");
        // Initialize the Apriltag Detection process
        initOp();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcam2, cameraMonitorViewId);

//        webcam.openCameraDevice();
        sleep(1000);
//        visionPortal.setActiveCamera(webcam2);  //causes OpenCvCameraException: setActiveCamera() called, but camera device is not opened!
//        visionPortal.setProcessorEnabled(RTDP, true);
//        FtcDashboard.getInstance().startCameraStream(RTDP, 0);

        // Establish Min and Max Gains and Exposure.  Then set a low exposure with high gain
        getCameraSetting();
//        myExposure = Math.min(5, minExposure);
        myExposure = 3;
        myGain = maxGain;
        setManualExposure(myExposure, myGain);
        //reset PT to zero
        setPanTiltZoom(0, 0, 0);

        // Wait for the match to begin.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Find lowest Exposure that gives reliable detection.");
            telemetry.addLine("Use DPAD L/R to adjust Pan.");
            telemetry.addLine("Use DPAD U/D to adjust Tilt.");
            telemetry.addLine("Use L Stick U/D to adjust Zoom.");
            telemetry.addLine("Use Left bump/trig to adjust Exposure.");
            telemetry.addLine("Use Right bump/trig to adjust Gain.\n");

//            if (RTDP.getDetectedPosition() == null) {
//                telemetry.addData("Status", "where is the tape :(");
//                telemetry.update();
//            }

            RobotLog.dd("RBR", "Vision: Exposure (ms) current = %d, max = %d, min = %d, Gain current = %d, max = %d, min = %d",
                    myExposure, maxExposure, minExposure, myGain, maxGain, minGain);
            RobotLog.dd("RBR", "Vision: minTilt = %d, maxTilt = %d, currentTilt = %d, minPan = %d, maxPan = %d, currentPan = %d, minZoom = %d, maxZoom = %d, currentZoom = %d",
                    minTilt, maxTilt, myTilt, minPan, maxPan, myPan, minZoom, maxZoom, myZoom);
            telemetry.addData("Pan", "%d  (%d - %d)", myPan, minPan, maxPan);
            telemetry.addData("Tilt", "%d  (%d - %d)", myTilt, minTilt, maxTilt);
            telemetry.addData("Zoom", "%d  (%d - %d)", myZoom, minZoom, maxZoom);
            telemetry.addData("Exposure", "%d  (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain", "%d  (%d - %d)", myGain, minGain, maxGain);
            telemetry.update();

            // check to see if we need to change exposure or gain.
            thisExpUp = gamepad1.left_bumper;
            thisExpDn = gamepad1.left_trigger > 0.25;
            thisGainUp = gamepad1.right_bumper;
            thisGainDn = gamepad1.right_trigger > 0.25;

            thisPanUp = gamepad1.dpad_left;
            thisPanDn = gamepad1.dpad_right;
            thisTiltUp = gamepad1.dpad_up;
            thisTiltDn = gamepad1.dpad_down;

            thisZoomDn = gamepad1.left_stick_y >= 0.25;
            thisZoomUp = gamepad1.left_stick_y <= -0.25;

            // look for clicks to change exposure
            if (thisExpUp && !lastExpUp) {
                myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            } else if (thisExpDn && !lastExpDn) {
                myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            }

            // look for clicks to change the gain
            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 1, minGain, maxGain);
                setManualExposure(myExposure, myGain);
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 1, minGain, maxGain);
                setManualExposure(myExposure, myGain);
            }

            int panTiltFactor = 10000;

            if (thisPanUp && !lastPanUp) {
                myPan = Range.clip(myPan + panTiltFactor, minPan, maxPan);
                setPanTiltZoom(myPan, myTilt, myZoom);
            } else if (thisPanDn && !lastPanDn) {
                myPan = Range.clip(myPan - panTiltFactor, minPan, maxPan);
                setPanTiltZoom(myPan, myTilt, myZoom);
            }

            if (thisTiltUp && !lastTiltUp) {
                myTilt = Range.clip(myTilt + panTiltFactor, minTilt, maxTilt);
                setPanTiltZoom(myPan, myTilt, myZoom);
            } else if (thisTiltDn && !lastTiltDn) {
                myTilt = Range.clip(myTilt - panTiltFactor, minTilt, maxTilt);
                setPanTiltZoom(myPan, myTilt, myZoom);
            }

            if (thisZoomUp && !lastZoomUp) {
                myZoom = Range.clip(myZoom + 1, minZoom, maxZoom);
                setPanTiltZoom(myPan, myTilt, myZoom);
            } else if (thisZoomDn && !lastZoomDn) {
                myZoom = Range.clip(myZoom - 1, minZoom, maxZoom);
                setPanTiltZoom(myPan, myTilt, myZoom);
            }

            lastExpUp = thisExpUp;
            lastExpDn = thisExpDn;
            lastGainUp = thisGainUp;
            lastGainDn = thisGainDn;

            lastPanUp = thisPanUp;
            lastPanDn = thisPanDn;
            lastTiltUp = thisTiltUp;
            lastTiltDn = thisTiltDn;
            lastZoomUp = thisZoomUp;
            lastZoomDn = thisZoomDn;

            //Pan + moves to the right, - moves to the left

            sleep(20);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initOp() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        DebugLog.log("webcam2 set from hardware map");
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

//        CustomPropLocationDetector customPropLocationDetector = new CustomPropLocationDetector(hardwareMap);

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam2)
                .build();
//        visionPortal.setProcessorEnabled(RTDP, true);
//        FtcDashboard.getInstance().startCameraStream(RTDP, 0);
    }

    private boolean setPanTiltZoom(int pan, int tilt, int zoom) {
        if (visionPortal == null) {
            return false;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            PtzControl.PanTiltHolder currentPanTilt = ptzControl.getPanTilt();
            currentPanTilt.pan = pan;
            currentPanTilt.tilt = tilt;
            ptzControl.setPanTilt(currentPanTilt);
            ptzControl.setZoom(zoom);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Manually set the camera gain and exposure.
        Can only be called AFTER calling initAprilTag();
        Returns true if controls are set.
     */
    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            return false;
        }

        // Wait for the camera to be open
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
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }

    /*
        Read this camera's minimum and maximum Exposure and Gain settings.
        Can only be called AFTER calling initAprilTag();
     */
    private void getCameraSetting() {
        // Ensure Vision Portal has been setup.
        if (visionPortal == null) {
            DebugLog.log("getCameraSetting - visionPortal is null");
            return;
        }

        // Wait for the camera to be open
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Get camera control values unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            ExposureControl.Mode mode = exposureControl.getMode();
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();

            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            PtzControl.PanTiltHolder currentPanTile = ptzControl.getPanTilt();
            maxPan = ptzControl.getMaxPanTilt().pan;
            maxTilt = ptzControl.getMaxPanTilt().tilt;
            maxZoom = ptzControl.getMaxZoom();
            minPan = ptzControl.getMinPanTilt().pan;
            minTilt = ptzControl.getMinPanTilt().tilt;
            minZoom = ptzControl.getMinZoom();

            myPan = currentPanTile.pan;
            myTilt = currentPanTile.tilt;
            myZoom = ptzControl.getZoom();

            RobotLog.dd("RBR", "Vision: Initial - minTilt = %d, maxTilt = %d, currentTilt = %d, minPan = %d, maxPan = %d, currentPan = %d, minZoom = %d, maxZoom = %d, currentZoom = %d",
                    minTilt, maxTilt, myTilt, minPan, maxPan, myPan, minZoom, maxZoom, myZoom);
            RobotLog.dd("RBR", "Vision: Initial - expMode = %s, exposure (ms) = %d, gain = %d",
                    mode.toString(), exposureControl.getExposure(TimeUnit.MILLISECONDS), gainControl.getGain());
            RobotLog.dd("RBR", "Vision: Initial - minExp = %d, maxExp = %d, minGain = %d, maxGain = %d",
                    minExposure, maxExposure, minGain, maxGain);
        }
    }
}
