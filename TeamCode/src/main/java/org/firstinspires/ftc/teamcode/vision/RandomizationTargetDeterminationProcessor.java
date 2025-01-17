package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.model.Alliance;
import org.firstinspires.ftc.teamcode.model.RandomizationItem;
import org.firstinspires.ftc.teamcode.model.TargetPosition;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;
@Config
public class RandomizationTargetDeterminationProcessor implements VisionProcessor, CameraStreamSource {

    //Note: refer to https://github.com/OpenFTC/EasyOpenCV for examples of OpenCV pipelines

    //these values are based on 640x480 resolution with near vertical camera position - adjust accordingly if resolution or camera angles change
    private static final int SIDE_REGION_WIDTH = 140;
    private static final int TOP_REGION_HEIGHT = 180;
    private static final int SKEW_OFFSET = 20;  //extra buffer to account for left/right skewing of spike marks (eg. appearing as / or \ instead of | )

    //Note: pixel coordinate plane: origin (0,0) is top left corner of image, x increases to the right, y increases down, bottom right corner = (width - 1, height - 1)

    //For sending OpenCV image Matrix frames to FTC Dashboard as Bitmaps
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final AtomicReference<Mat> lastMatFrame = new AtomicReference<>(new Mat());

    //rectangular regions on the screen/frame we will look in for spike marks to determine target position
    private Rect leftRegion;
    private Rect centerRegion;
    private Rect rightRegion;
    private Mat hsvMat = new Mat();  //will contain HSV version of input image
    private Mat colorContoursMat = new Mat();  //will contain filtered color pixels
    private Mat colorContoursHierarchyMat = new Mat(); //will contain hierarchy of color contours
    private Mat redMask1 = new Mat(); //will contain red color range 1 mask
    private Mat redMask2 = new Mat(); //will contain red color range 2 mask

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

    public static double MIN_RECTANGLE_AREA_THRESHOLD = 100.00; //test and adjust - min color blob bounding rectangle area we care about in pixels
    private final Alliance alliance;
    private final RandomizationItem randomizationItem;
    private final Telemetry telemetry;
    private boolean stopLookingWhenTargetFound = true;
    private volatile Recognition pixelRecognition; //TFOD detected randomization Pixel lying on spike mark
    private volatile TargetPosition detectedPosition;

    public RandomizationTargetDeterminationProcessor(Alliance alliance, RandomizationItem randomizationItem, Telemetry telemetry) {
        this.alliance = alliance;
        this.randomizationItem = randomizationItem;
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        telemetry.addData("Vision",  "RTDP initial W = %d, H = %d, seeking %s spike marks", width, height, alliance);
        telemetry.update();
        RobotLog.dd("RBR","RTDP init W %d, H = %d, alliance = %s", width, height, alliance.toString());
        RobotLog.dd("RBR", "RTDP init detectedPosition = ", String.valueOf(detectedPosition));
        int topRegionWidth = width - (2 * SIDE_REGION_WIDTH);
        //leftRegion = new Rect(new Point(0, TOP_REGION_HEIGHT), new Point(SIDE_REGION_WIDTH + SKEW_OFFSET, height - 1));
        //centerRegion = new Rect(new Point(SIDE_REGION_WIDTH - SKEW_OFFSET, 0), new Point(SIDE_REGION_WIDTH + topRegionWidth + SKEW_OFFSET, TOP_REGION_HEIGHT));
        //rightRegion = new Rect(new Point(width - SIDE_REGION_WIDTH - SKEW_OFFSET, TOP_REGION_HEIGHT), new Point(width - 1, height - 1));

        leftRegion = new Rect(new Point(0, 0), new Point(SIDE_REGION_WIDTH, height - 1));
        centerRegion = new Rect(new Point(SIDE_REGION_WIDTH, 0), new Point(SIDE_REGION_WIDTH + topRegionWidth, height - 1));
        rightRegion = new Rect(new Point(width - SIDE_REGION_WIDTH, 0), new Point(width - 1, height - 1));

        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        lastMatFrame.set(new Mat());
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        List<Rect> spikeBoundingRectangles = Collections.emptyList();
        colorContoursMat = new Mat();

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_BGR2HSV);  //convert to HSV color space for better color detection
        if (alliance != Alliance.BLUE) { //TODO: FIND A REAL SOLUTION TO THE COLOR PROBLEM
            Core.inRange(hsvMat, ColorRange.BLUE_LOWER_RANGE, ColorRange.BLUE_UPPER_RANGE, colorContoursMat); //select only blue pixels
        } else {
            //red color range wraps around, so we need to merge two masks
            Core.inRange(hsvMat, ColorRange.RED_LOWER_RANGE_1, ColorRange.RED_UPPER_RANGE_1, redMask1); //red pixels 1
            Core.inRange(hsvMat, ColorRange.RED_LOWER_RANGE_2, ColorRange.RED_UPPER_RANGE_2, redMask2); //red pixels 2
            Core.bitwise_or(redMask1, redMask2, colorContoursMat); //combine masks to select all red pixels covering both color ranges
        }

        //find color contours / blobs
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(colorContoursMat, contours, colorContoursHierarchyMat, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        colorContoursMat.release();
        for (MatOfPoint contour : contours) {
            MatOfPoint2f areaPoints = new MatOfPoint2f(contour.toArray());
            RotatedRect boundingRect = Imgproc.minAreaRect(areaPoints);

            double rectangleArea = boundingRect.size.area();

            //ignore blobs smaller than our minimum containing rectangle area
            if (rectangleArea > MIN_RECTANGLE_AREA_THRESHOLD) {
                if (spikeBoundingRectangles.isEmpty()) {
                    spikeBoundingRectangles = new ArrayList<>();
                }

                Point boundingRectPoints[] = new Point[4];
                boundingRect.points(boundingRectPoints);
                Rect bbox = Imgproc.boundingRect(new MatOfPoint(boundingRectPoints));
                spikeBoundingRectangles.add(bbox);
            }

            //don't leak native allocated memory
            areaPoints.release();
            contour.release();
        }

        if (!spikeBoundingRectangles.isEmpty()) {
            telemetry.addData("Vision",  "RTDP found %d bounding rectangles for spike marks", spikeBoundingRectangles.size());
            telemetry.update();
        }

        return spikeBoundingRectangles;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint targetDetectionRectanglePaint = new Paint();
        targetDetectionRectanglePaint.setColor(Color.MAGENTA);
        targetDetectionRectanglePaint.setStyle(Paint.Style.STROKE);
        targetDetectionRectanglePaint.setStrokeWidth(scaleCanvasDensity * 4);
        android.graphics.Rect leftRegionAndroidRect = toAndroidGraphicsRect(leftRegion, scaleBmpPxToCanvasPx);
        android.graphics.Rect centerRegionAndroidRect = toAndroidGraphicsRect(centerRegion, scaleBmpPxToCanvasPx);
        android.graphics.Rect rightRegionAndroidRect = toAndroidGraphicsRect(rightRegion, scaleBmpPxToCanvasPx);

        //draw magenta rectangles around target detection regions
        canvas.drawRect(leftRegionAndroidRect, targetDetectionRectanglePaint);
        canvas.drawRect(centerRegionAndroidRect, targetDetectionRectanglePaint);
        canvas.drawRect(rightRegionAndroidRect, targetDetectionRectanglePaint);

        List<Rect> spikeBoundingRectangles = (List<Rect>) userContext; //userContext contains return value of processFrame()
        List<android.graphics.Rect> androidGraphicsSpikeBoundingRectangles = spikeBoundingRectangles.stream()
                .map(rect -> toAndroidGraphicsRect(rect, scaleBmpPxToCanvasPx)).collect(Collectors.toList());
        Paint spikeRectanglePaint = new Paint();
        spikeRectanglePaint.setColor(Color.GREEN);
        spikeRectanglePaint.setStyle(Paint.Style.STROKE);
        spikeRectanglePaint.setStrokeWidth(scaleCanvasDensity * 4);
        Map<TargetPosition, Integer> candidateSpikeRectCountByPosition = new EnumMap<>(TargetPosition.class);

        if (!androidGraphicsSpikeBoundingRectangles.isEmpty()) {
            candidateSpikeRectCountByPosition.put(TargetPosition.LEFT, 0);
            candidateSpikeRectCountByPosition.put(TargetPosition.CENTER, 0);
            candidateSpikeRectCountByPosition.put(TargetPosition.RIGHT, 0);
        }

        for (android.graphics.Rect androidGraphicsSpikeBoundingRectangle : androidGraphicsSpikeBoundingRectangles) {
            //draw green rectangles around detected spike marks
            canvas.drawRect(androidGraphicsSpikeBoundingRectangle, spikeRectanglePaint);
            if (detectedPosition == null) {
                //check which detection region of the screen the bounding rectangle falls within
                if (leftRegionAndroidRect.contains(androidGraphicsSpikeBoundingRectangle) || leftRegionAndroidRect.intersect(androidGraphicsSpikeBoundingRectangle)) {
                    candidateSpikeRectCountByPosition.put(TargetPosition.LEFT, candidateSpikeRectCountByPosition.get(TargetPosition.LEFT) + 1);
                } else if (centerRegionAndroidRect.contains(androidGraphicsSpikeBoundingRectangle) || centerRegionAndroidRect.intersect(androidGraphicsSpikeBoundingRectangle)) {
                    candidateSpikeRectCountByPosition.put(TargetPosition.CENTER, candidateSpikeRectCountByPosition.get(TargetPosition.CENTER) + 1);
                } else if (rightRegionAndroidRect.contains(androidGraphicsSpikeBoundingRectangle) || rightRegionAndroidRect.intersect(androidGraphicsSpikeBoundingRectangle)) {
                    candidateSpikeRectCountByPosition.put(TargetPosition.RIGHT, candidateSpikeRectCountByPosition.get(TargetPosition.RIGHT) + 1);
                }
            }

            /*
            //check if the pixel is on a spike mark
            if (pixelRecognition != null && detectedPosition == null) {
                android.graphics.Rect pixelBoundingRectangle = new android.graphics.Rect(Math.round(pixelRecognition.getLeft() * scaleBmpPxToCanvasPx),
                        Math.round(pixelRecognition.getTop() * scaleBmpPxToCanvasPx),
                        Math.round(pixelRecognition.getRight() * scaleBmpPxToCanvasPx),
                        Math.round(pixelRecognition.getBottom() * scaleBmpPxToCanvasPx));
                if (pixelBoundingRectangle.intersect(androidGraphicsSpikeBoundingRectangle)) {
                    //we found the pixel on a spike mark
                    if (leftRegionAndroidRect.contains(pixelBoundingRectangle) || leftRegionAndroidRect.intersect(pixelBoundingRectangle)) {
                        detectedPosition = TargetPosition.LEFT;
                    } else if (centerRegionAndroidRect.contains(pixelBoundingRectangle) || centerRegionAndroidRect.intersect(pixelBoundingRectangle)) {
                        detectedPosition = TargetPosition.CENTER;
                    } else if (rightRegionAndroidRect.contains(pixelBoundingRectangle) || rightRegionAndroidRect.intersect(pixelBoundingRectangle)) {
                        detectedPosition = TargetPosition.RIGHT;
                    }

                    telemetry.addData("Vision",  "Target scoring position detected: %s", detectedPosition);
                    telemetry.update();

                    //TODO: how to place the purple scoring pixel on the same spike mark relative to the detected white pixel?
                    // maybe pre-program arm/drive sequence to place pixel on a static pre-determined position on each TargetPosition spike mark?
                }
            }
             */
        }

        if (!stopLookingWhenTargetFound || (detectedPosition == null && !androidGraphicsSpikeBoundingRectangles.isEmpty())) {
            Map.Entry<TargetPosition, Integer> targetEntry;

            boolean allSpikesDetected = candidateSpikeRectCountByPosition.entrySet().stream().allMatch(entry -> entry.getValue() > 0);
            if (allSpikesDetected) {
                if (randomizationItem == RandomizationItem.PIXEL) {
                    //consider the position with the most detected rectangles the target (the white pixel should cause the single rect to break into multiple)
                    targetEntry = Collections.max(candidateSpikeRectCountByPosition.entrySet(), Map.Entry.comparingByValue());
                } else {
                    //consider the position with the least detected rectangles the target (the colored prop should NOT cause the single rect to break into multiple)
                    //TODO: test this assumption
                    targetEntry = Collections.min(candidateSpikeRectCountByPosition.entrySet(), Map.Entry.comparingByValue());
                }

                detectedPosition = targetEntry.getKey();
                telemetry.addData("Vision", "Target scoring position detected: %s with %d rects", detectedPosition, targetEntry.getValue());
                telemetry.update();
                RobotLog.dd("RBR", "Vision - target scoring position detected %s with %d rects", detectedPosition, targetEntry.getValue());
            }
        }

        //convert OpenCV image Matrix to Android Bitmap for use in FTC dashboard
        Bitmap b = Bitmap.createBitmap(onscreenWidth, onscreenHeight, Bitmap.Config.RGB_565);
        Mat lastMatRef = lastMatFrame.get();
        Utils.matToBitmap(lastMatRef, b);
        lastFrame.set(b);
        lastMatRef.release();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        //send the frame bitmap to FTC dashboard
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    //convert opencv Rect to android.graphics.Rect
    private android.graphics.Rect toAndroidGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public TargetPosition getDetectedPosition() {
        return detectedPosition;
    }

    public Recognition getPixelRecognition() {
        return pixelRecognition;
    }

    public void setDetectedPosition(TargetPosition detectedPosition) {
        this.detectedPosition = detectedPosition;
    }

    public void setPixelRecognition(Recognition pixelRecognition) {
        this.pixelRecognition = pixelRecognition;
    }

    public void setStopLookingWhenTargetFound(boolean stopLookingWhenTargetFound) {
        this.stopLookingWhenTargetFound = stopLookingWhenTargetFound;
    }
}
