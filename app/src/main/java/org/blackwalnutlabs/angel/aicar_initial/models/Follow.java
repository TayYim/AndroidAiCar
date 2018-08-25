package org.blackwalnutlabs.angel.aicar_initial.models;

import org.blackwalnutlabs.angel.aicar_initial.setting.ImageSetting;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.blackwalnutlabs.angel.aicar_initial.setting.ImageSetting.MAXHEIGHT;
import static org.blackwalnutlabs.angel.aicar_initial.setting.ImageSetting.MAXWIDTH;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.RETR_TREE;

public class Follow {
    private static final String TAG = "Follow";

    private Mat[] tmpMats;
    private Mat emptyMat;
    private Mat zeroMat;

    private MatOfPoint[] tmpMatOfPoints;
    private MatOfPoint emptyMatOfPoint;

    private Mat kernel;

    public Follow(Map<String, Object> tmpMap, Map<String, Object> funMap, Map<String, Object> othersMap) {
        tmpMats = (Mat[]) tmpMap.get("Mat");
        tmpMatOfPoints = (MatOfPoint[]) tmpMap.get("MatOfPoint");

        emptyMat = (Mat) funMap.get("EmptyMat");
        emptyMatOfPoint = (MatOfPoint) funMap.get("EmptyMatOfPoint");
        zeroMat = (Mat) funMap.get("ZeroMat");

        kernel = (Mat) othersMap.get("kernel");
    }

    public int process(Mat src, Mat dst){
        src.copyTo(dst);

        Core.line(dst, new Point(0, MAXHEIGHT), new Point(MAXWIDTH / 2 - MAXWIDTH / 8, MAXHEIGHT / 2 - MAXHEIGHT / 9), new Scalar(255, 255, 255), 4);
        Core.line(dst, new Point(MAXWIDTH, MAXHEIGHT), new Point(MAXWIDTH / 2 + MAXWIDTH / 8, MAXHEIGHT / 2 - MAXHEIGHT / 9), new Scalar(255, 255, 255), 4);
        Core.line(dst, new Point(MAXWIDTH / 2 + MAXWIDTH / 8, MAXHEIGHT / 2 - MAXHEIGHT / 9), new Point(MAXWIDTH / 2 - MAXWIDTH / 8, MAXHEIGHT / 2 - MAXHEIGHT / 9), new Scalar(255, 255, 255), 4);


        return 0; //ahead
    }
}
