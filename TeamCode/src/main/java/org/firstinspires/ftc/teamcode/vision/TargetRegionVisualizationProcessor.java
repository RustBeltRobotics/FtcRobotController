package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

/**
 * Image processor used to visualize the target detection regions on the screen
 */
public class TargetRegionVisualizationProcessor implements VisionProcessor {

//    private static final int SIDE_REGION_WIDTH = 140;  //this value is for 640x480 resolution
    private static final int SIDE_REGION_WIDTH = 350;  //this value is for 1280x720 resolution

    //rectangular regions on the screen/frame we will look in for spike marks to determine target position
    private Rect leftRegion;
    private Rect centerRegion;
    private Rect rightRegion;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        int topRegionWidth = width - (2 * SIDE_REGION_WIDTH);
        leftRegion = new Rect(new Point(0, 0), new Point(SIDE_REGION_WIDTH, height - 1));
        centerRegion = new Rect(new Point(SIDE_REGION_WIDTH, 0), new Point(SIDE_REGION_WIDTH + topRegionWidth, height - 1));
        rightRegion = new Rect(new Point(width - SIDE_REGION_WIDTH, 0), new Point(width - 1, height - 1));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
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

        //draw magenta rectangles around target detection regions
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
}
