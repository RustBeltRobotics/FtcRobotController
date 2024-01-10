package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.teamcode.util.DebugLog;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

@Config
public class CustomPropLocationDetectorTFOD {

//    public static double PROP_OBJECT_MAX_AREA = 16000.00; //(160x100) in 640x480 resolution
public static double PROP_OBJECT_MAX_AREA = 62500.00; //(250x250) in 1280x720 resolution

    private static final String TFOD_MODEL_ASSET = "efficientdet_lite0.tflite";
    private static final String[] LABELS = {
            "person",
            "bicycle",
            "car",
            "motorcycle",
            "airplane",
            "bus",
            "train",
            "truck",
            "boat",
            "traffic light",
            "fire hydrant",
            "stop sign",
            "parking meter",
            "bench",
            "bird",
            "cat",
            "dog",
            "horse",
            "sheep",
            "cow",
            "elephant",
            "bear",
            "zebra",
            "giraffe",
            "backpack",
            "umbrella",
            "handbag",
            "tie",
            "suitcase",
            "frisbee",
            "skis",
            "snowboard",
            "sports ball",
            "kite",
            "baseball bat",
            "baseball glove",
            "skateboard",
            "surfboard",
            "tennis racket",
            "bottle",
            "wine glass",
            "cup",
            "fork",
            "knife",
            "spoon",
            "bowl",
            "banana",
            "apple",
            "sandwich",
            "orange",
            "broccoli",
            "carrot",
            "hot dog",
            "pizza",
            "donut",
            "cake",
            "chair",
            "couch",
            "potted plant",
            "bed",
            "dining table",
            "toilet",
            "tv",
            "laptop",
            "mouse",
            "remote",
            "keyboard",
            "cell phone",
            "microwave",
            "oven",
            "toaster",
            "sink",
            "refrigerator",
            "book",
            "clock",
            "vase",
            "scissors",
            "teddy bear",
            "hair drier",
            "toothbrush"
    };

    //assuming 640x480 viewport
//    private android.graphics.Rect leftRegion = new android.graphics.Rect(0, 0,  140, 479);  //140 width
//    private android.graphics.Rect centerRegion = new android.graphics.Rect(141, 0, 499, 479);  //358 width
//    private android.graphics.Rect rightRegion = new android.graphics.Rect(500, 0, 639, 479); //139 width
    //assuming 1280x720 viewport
    private android.graphics.Rect leftRegion = new android.graphics.Rect(0, 0,  350, 719);  //350 width
    private android.graphics.Rect centerRegion = new android.graphics.Rect(351, 0, 930, 719);  //580 width
    private android.graphics.Rect rightRegion = new android.graphics.Rect(931, 0, 1279, 719); //350 width

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    public CustomPropLocationDetectorTFOD(HardwareMap hardwareMap) {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //TODO: test using 1280x720 resolution
//        builder.setCameraResolution(new Size(640, 480));
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set and enable the processors
        builder.addProcessors(tfod, new TargetRegionVisualizationProcessor());

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions
        tfod.setMinResultConfidence(0.25f);
    }

    public TargetPosition detectPropLocation() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (!currentRecognitions.isEmpty()) {
            List<Recognition> minAreaRecognitions = currentRecognitions.stream().filter(r -> (r.getWidth() * r.getHeight()) <= PROP_OBJECT_MAX_AREA).collect(Collectors.toList());
            Recognition selectedRecognition = null;
            if (minAreaRecognitions.isEmpty()) {
                DebugLog.log("Vision - no recognition with area below threshold");
                currentRecognitions.stream().forEach(r -> DebugLog.log("Vision - ignored candidate recognition '%s' - confidence = %.1f, width = %.1f, height = %.1f",
                        r.getLabel(), r.getConfidence() * 100, r.getWidth(), r.getHeight()));

                return null;
            } else if (minAreaRecognitions.size() == 1) {
                selectedRecognition = minAreaRecognitions.get(0);
            } else {
                //if multiple recognitions, select the one with the highest confidence
                selectedRecognition = minAreaRecognitions.stream().max(Comparator.comparing(r -> r.getConfidence())).get();
            }

            TargetPosition targetPosition = getTargetPositionForRecognition(selectedRecognition);
            DebugLog.log("Vision - found recognition '%s' with %.1f confidence, width = %.1f, height = %.1f, target position = %s",
                    selectedRecognition.getLabel(), selectedRecognition.getWidth(), selectedRecognition.getHeight(),
                    selectedRecognition.getConfidence() * 100, targetPosition);

            return targetPosition;
        }

        return null;
    }

    private TargetPosition getTargetPositionForRecognition(Recognition recognition) {
        android.graphics.Rect boundingRectangle = toAndroidGraphicsRect(recognition);
        if (leftRegion.contains(boundingRectangle) || leftRegion.intersect(boundingRectangle)) {
            return TargetPosition.LEFT;
        } else if (centerRegion.contains(boundingRectangle) || centerRegion.intersect(boundingRectangle)) {
            return TargetPosition.CENTER;
        } else if (rightRegion.contains(boundingRectangle) || rightRegion.intersect(boundingRectangle)) {
            return TargetPosition.RIGHT;
        }

        return null;
    }

    private android.graphics.Rect toAndroidGraphicsRect(Recognition recognition) {
        int left = Math.round(recognition.getLeft());
        int top = Math.round(recognition.getTop());
        int right = Math.round(recognition.getRight());
        int bottom = top + Math.round(recognition.getBottom());

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public void panLeft() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            PtzControl.PanTiltHolder currentPanTilt = ptzControl.getPanTilt();
            currentPanTilt.pan = ptzControl.getMinPanTilt().tilt;
            ptzControl.setPanTilt(currentPanTilt);
        }
    }

    public void resetPanTilt() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            PtzControl.PanTiltHolder currentPanTilt = ptzControl.getPanTilt();
            currentPanTilt.pan = 0;
            currentPanTilt.tilt = 0;
            ptzControl.setPanTilt(currentPanTilt);
        }
    }

    public void panRight() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
            PtzControl.PanTiltHolder currentPanTilt = ptzControl.getPanTilt();
            currentPanTilt.pan = ptzControl.getMaxPanTilt().tilt;
            ptzControl.setPanTilt(currentPanTilt);
        }
    }

    public void shutdown() {
        tfod.shutdown();
        visionPortal.close();
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }
}
