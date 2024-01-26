package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.model.Alliance;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.Collections;

@Config
public class CustomPropLocationDetectorOpenCV implements VisionProcessor {

    private Rect leftRegion;
    private Rect centerRegion;
    private Rect rightRegion;
    private Mat leftRegionMat = new Mat();
    private Mat centerRegionMat = new Mat();
    private Mat rightRegionMat = new Mat();
    private Mat hsvMat = new Mat();  //will contain HSV version of input image
    private Mat colorMaskedMat = new Mat();  //will contain filtered color pixels
    private Mat redMask1 = new Mat(); //will contain red color range 1 mask
    private Mat redMask2 = new Mat(); //will contain red color range 2 mask
    private VisionPortal visionPortal;

    private final Alliance alliance;
    private final Telemetry telemetry;
    private volatile TargetPosition detectedPosition;

    static class ColorRange {
        static final Scalar BLUE_LOWER_RANGE = new Scalar(100, 150, 0);
        static final Scalar BLUE_UPPER_RANGE = new Scalar(140, 255, 255);
        //static final Scalar RED_LOWER_RANGE_1 = new Scalar(0, 70, 50);
        static final Scalar RED_LOWER_RANGE_1 = new Scalar(0, 40, 50);
        static final Scalar RED_UPPER_RANGE_1 = new Scalar(10, 255, 255);
        //static final Scalar RED_LOWER_RANGE_2 = new Scalar(170, 70, 50);
        static final Scalar RED_LOWER_RANGE_2 = new Scalar(170, 40, 50);
        static final Scalar RED_UPPER_RANGE_2 = new Scalar(180, 255, 255);
    }

    public CustomPropLocationDetectorOpenCV(Alliance alliance, Telemetry telemetry) {
        this.alliance = alliance;
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        //Preferred resolution = 1280x720
        Size cameraResolution = new Size(width, height);
        int sixteenths = cameraResolution.getWidth() / 16;
        int sideWidth = sixteenths * 4;
        int centerWidth = sixteenths * 8;

        leftRegion = new Rect(new Point(0, 0), new Point(sideWidth, height - 1));
        centerRegion = new Rect(new Point(sideWidth, 0), new Point(centerWidth + sideWidth, height - 1));
        rightRegion = new Rect(new Point(centerWidth + sideWidth, 0), new Point(width - 1, height - 1));

        telemetry.addData("Vision",  "CustomPropLocationDetectorOpenCV initial W = %d, H = %d, alliance = %s", width, height, alliance.toString());
        telemetry.update();
        RobotLog.dd("RBR","CustomPropLocationDetectorOpenCV init W %d, H = %d, alliance = %s", width, height, alliance.toString());
        RobotLog.dd("RBR", "CustomPropLocationDetectorOpenCV init detectedPosition = ", String.valueOf(detectedPosition));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_BGR2HSV);  //convert to HSV color space for better color detection
        if (alliance != Alliance.BLUE) { //TODO: FIND A REAL SOLUTION TO THE COLOR PROBLEM
            Core.inRange(hsvMat, CustomPropLocationDetectorOpenCV.ColorRange.BLUE_LOWER_RANGE, CustomPropLocationDetectorOpenCV.ColorRange.BLUE_UPPER_RANGE, colorMaskedMat); //select only blue pixels
        } else {
            //red color range wraps around, so we need to merge two masks
            Core.inRange(hsvMat, CustomPropLocationDetectorOpenCV.ColorRange.RED_LOWER_RANGE_1, CustomPropLocationDetectorOpenCV.ColorRange.RED_UPPER_RANGE_1, redMask1); //red pixels 1
            Core.inRange(hsvMat, CustomPropLocationDetectorOpenCV.ColorRange.RED_LOWER_RANGE_2, CustomPropLocationDetectorOpenCV.ColorRange.RED_UPPER_RANGE_2, redMask2); //red pixels 2
            Core.bitwise_or(redMask1, redMask2, colorMaskedMat); //combine masks to select all red pixels covering both color ranges
        }

        int numRows = colorMaskedMat.rows();
        int numCols = colorMaskedMat.cols();
        int sixteenths = numCols / 16;
        int sideWidth = sixteenths * 4;
        int centerWidth = sixteenths * 8;

        leftRegionMat = colorMaskedMat.submat(0, numRows, 0, sideWidth);
        centerRegionMat = colorMaskedMat.submat(0, numRows, sideWidth, centerWidth + sideWidth);
        rightRegionMat = colorMaskedMat.submat(0, numRows, centerWidth + sideWidth, numCols);
        int coloredPixelCountInLeft = Core.countNonZero(leftRegionMat);
        int coloredPixelCountInCenter = Core.countNonZero(centerRegionMat);
        int coloredPixelCountInRight = Core.countNonZero(rightRegionMat);
        int maxPixelCount = Collections.max(Arrays.asList(coloredPixelCountInLeft, coloredPixelCountInCenter, coloredPixelCountInRight));

        if (coloredPixelCountInLeft == maxPixelCount) {
            detectedPosition = TargetPosition.LEFT;
        } else if (coloredPixelCountInCenter == maxPixelCount) {
            detectedPosition = TargetPosition.CENTER;
        } else {
            detectedPosition = TargetPosition.RIGHT;
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint sideRectanglePaint = new Paint();
        sideRectanglePaint.setColor(Color.MAGENTA);
        sideRectanglePaint.setStyle(Paint.Style.STROKE);
        sideRectanglePaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint topRectanglePaint = new Paint();
        topRectanglePaint.setColor(Color.CYAN);
        topRectanglePaint.setStyle(Paint.Style.STROKE);
        topRectanglePaint.setStrokeWidth(scaleCanvasDensity * 4);

        android.graphics.Rect leftRegionAndroidRect = toAndroidGraphicsRect(leftRegion, scaleBmpPxToCanvasPx);
        android.graphics.Rect centerRegionAndroidRect = toAndroidGraphicsRect(centerRegion, scaleBmpPxToCanvasPx);
        android.graphics.Rect rightRegionAndroidRect = toAndroidGraphicsRect(rightRegion, scaleBmpPxToCanvasPx);

        //draw colored rectangles around target detection regions
        canvas.drawRect(leftRegionAndroidRect, sideRectanglePaint);
        canvas.drawRect(centerRegionAndroidRect, topRectanglePaint);
        canvas.drawRect(rightRegionAndroidRect, sideRectanglePaint);
    }

    //convert opencv Rect to android.graphics.Rect
    private android.graphics.Rect toAndroidGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public TargetPosition getDetectedPosition() {
        return detectedPosition;
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

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void setVisionPortal(VisionPortal visionPortal) {
        this.visionPortal = visionPortal;
    }

    public void setDetectedPosition(TargetPosition detectedPosition) {
        this.detectedPosition = detectedPosition;
    }
}
