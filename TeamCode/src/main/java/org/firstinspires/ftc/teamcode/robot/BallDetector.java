package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class BallDetector implements VisionProcessor {

    public enum BallType { POLLEN, NECTAR_RED, NECTAR_BLUE }

    public static final class Detection {
        public final BallType type;
        public final double fieldX, fieldY;
        public final double rangeIn, bearingDeg;
        public final double areaPx;

        Detection(BallType t, double fx, double fy, double r, double b, double a) {
            type = t; fieldX = fx; fieldY = fy; rangeIn = r; bearingDeg = b; areaPx = a;
        }
    }

    // ---- HSV gates, OpenCV units: H 0-179, S 0-255, V 0-255 --------------
    public static Scalar YELLOW_LO = new Scalar(21, 90, 95);
    public static Scalar YELLOW_HI = new Scalar(37, 255, 255);
    // Red wraps around 0, so it needs two windows.
    public static Scalar RED1_LO = new Scalar(0, 80, 100);
    public static Scalar RED1_HI = new Scalar(10, 255, 255);
    public static Scalar RED2_LO = new Scalar(163, 80, 100);
    public static Scalar RED2_HI = new Scalar(179, 255, 255);
    // Blue measured V 255 -- clipped, the camera is blowing this ball out.
    public static Scalar BLUE_LO = new Scalar(96, 75, 80);
    public static Scalar BLUE_HI = new Scalar(118, 255, 255);

    // Ball diameters, MEASURED 14 Nov.
    public static double POLLEN_DIAM_IN = 2.8;
    public static double NECTAR_DIAM_IN = 3.6;

    // Cheap pre-filter only.
    public static double MIN_AREA_PX = 1200;
    public static double MAX_AREA_PX = 60000;
    public static double MIN_CIRCULARITY = 0.55;

    // Size consistency.
    public static boolean SIZE_CHECK = true;
    public static double SIZE_TOL_LO = 0.35;
    public static double SIZE_TOL_HI = 2.50;

    public static double diameterIn(BallType t) {
        return t == BallType.POLLEN ? POLLEN_DIAM_IN : NECTAR_DIAM_IN;
    }

    // What a ball of this type SHOULD cover at this range.
    public double expectedAreaPx(BallType t, double rangeIn, double bearingDeg) {
        double diam = diameterIn(t);
        double g0 = rangeIn * Math.cos(Math.toRadians(bearingDeg));
        double g1 = rangeIn * Math.sin(Math.toRadians(bearingDeg));
        double dx = g0 + C270_BEHIND_IN, dy = g1 + C270_RIGHT_IN;
        double dz = C270_HEIGHT_IN - diam / 2.0;
        double slant = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (slant < 1e-6) return 0;
        double d = fx * diam / slant;
        return Math.PI / 4.0 * d * d;
    }

    // ---- camera, from RobotHardware --------------------------------------
    public static double C270_HEIGHT_IN = RobotHardware.C270_HEIGHT_IN;
    public static double C270_BEHIND_IN = RobotHardware.C270_BEHIND_IN;
    public static double C270_RIGHT_IN  = RobotHardware.C270_RIGHT_IN;
    public static double C270_PITCH_DEG = -RobotHardware.C270_PITCH_DEG;

    public static double C270_DIAGONAL_FOV_DEG = RobotHardware.C270_DIAGONAL_FOV_DEG;

    private int imgW = 640, imgH = 480;
    private double fx, fy, cx, cy;

    // robot pose, pushed in each loop so detections can be reported in field
    private volatile double poseX = 72, poseY = 72, poseYawDeg = 0;
    private volatile boolean poseValid = false;

    private final Object lock = new Object();
    private List<Detection> latest = new ArrayList<Detection>();
    private int pollenMaskPx = 0;

    // scratch, reused so the processor does not allocate every frame
    private final Mat hsv = new Mat(), m1 = new Mat(), m2 = new Mat(), mask = new Mat();

    private volatile double centreH = 0, centreS = 0, centreV = 0;
    public static int CENTRE_PATCH_PX = 40;

    public double[] getCentreHsv() { return new double[] { centreH, centreS, centreV }; }

    private volatile double centre0 = 0, centre1 = 0, centre2 = 0;

    public double[] getCentreRaw() { return new double[] { centre0, centre1, centre2 }; }

    public static boolean FRAME_IS_BGR = false;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        imgW = width; imgH = height;
        computeIntrinsics();
    }

    private void computeIntrinsics() {
        double diagPx = Math.hypot(imgW, imgH);
        double f = (diagPx / 2.0) / Math.tan(Math.toRadians(C270_DIAGONAL_FOV_DEG / 2.0));
        fx = f; fy = f;
        cx = imgW / 2.0;
        cy = imgH / 2.0;
    }

    public double hfovDeg() { return 2 * Math.toDegrees(Math.atan((imgW / 2.0) / fx)); }
    public double vfovDeg() { return 2 * Math.toDegrees(Math.atan((imgH / 2.0) / fy)); }

    public void setPose(double xIn, double yIn, double yawDeg) {
        poseX = xIn; poseY = yIn; poseYawDeg = yawDeg; poseValid = true;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv,
                FRAME_IS_BGR ? Imgproc.COLOR_BGR2HSV : Imgproc.COLOR_RGB2HSV);
        sampleCentre(frame);
        List<Detection> found = new ArrayList<Detection>();

        Core.inRange(hsv, YELLOW_LO, YELLOW_HI, mask);
        // Count the pixels BEFORE collect() applies its shape filter
        int pollenPx = Core.countNonZero(mask);
        collect(mask, BallType.POLLEN, found);

        Core.inRange(hsv, RED1_LO, RED1_HI, m1);
        Core.inRange(hsv, RED2_LO, RED2_HI, m2);
        Core.bitwise_or(m1, m2, mask);
        collect(mask, BallType.NECTAR_RED, found);

        Core.inRange(hsv, BLUE_LO, BLUE_HI, mask);
        collect(mask, BallType.NECTAR_BLUE, found);

        Collections.sort(found, new Comparator<Detection>() {
            @Override public int compare(Detection a, Detection b) {
                return Double.compare(a.rangeIn, b.rangeIn);
            }
        });
        // Only while accepting.
        synchronized (lock) {
            if (accepting) { latest = found; pollenMaskPx = pollenPx; }
        }
        return null;
    }

    private void sampleCentre(Mat raw) {
        int half = CENTRE_PATCH_PX / 2;
        int x = Math.max(0, imgW / 2 - half), y = Math.max(0, imgH / 2 - half);
        int w = Math.min(CENTRE_PATCH_PX, imgW - x), h = Math.min(CENTRE_PATCH_PX, imgH - y);
        if (w <= 0 || h <= 0) return;

        Rect r = new Rect(x, y, w, h);

        Mat roi = hsv.submat(r);
        Scalar mean = Core.mean(roi);
        roi.release();
        centreH = mean.val[0]; centreS = mean.val[1]; centreV = mean.val[2];

        // The delivered frame may carry an alpha channel; only the first three
        Mat rawRoi = raw.submat(r);
        Scalar rawMean = Core.mean(rawRoi);
        rawRoi.release();
        centre0 = rawMean.val[0]; centre1 = rawMean.val[1]; centre2 = rawMean.val[2];
    }

    public static void setGate(BallType t, double hLo, double hHi, double sMin, double vMin) {
        switch (t) {
            case POLLEN:
                YELLOW_LO = new Scalar(hLo, sMin, vMin);
                YELLOW_HI = new Scalar(hHi, 255, 255);
                break;
            case NECTAR_RED:
                // Red wraps 0, so the low window anchors at 0 and the high
                RED1_LO = new Scalar(0, sMin, vMin);
                RED1_HI = new Scalar(hLo, 255, 255);
                RED2_LO = new Scalar(hHi, sMin, vMin);
                RED2_HI = new Scalar(179, 255, 255);
                break;
            case NECTAR_BLUE:
                BLUE_LO = new Scalar(hLo, sMin, vMin);
                BLUE_HI = new Scalar(hHi, 255, 255);
                break;
        }
    }

    public static double[] getGate(BallType t) {
        switch (t) {
            case POLLEN:      return new double[] { YELLOW_LO.val[0], YELLOW_HI.val[0],
                                                    YELLOW_LO.val[1], YELLOW_LO.val[2] };
            case NECTAR_RED:  return new double[] { RED1_HI.val[0], RED2_LO.val[0],
                                                    RED1_LO.val[1], RED1_LO.val[2] };
            default:          return new double[] { BLUE_LO.val[0], BLUE_HI.val[0],
                                                    BLUE_LO.val[1], BLUE_LO.val[2] };
        }
    }

    private void collect(Mat binary, BallType type, List<Detection> out) {
        // Open then close: kill speckle, then fill the holes in a perforated
        Mat k = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_OPEN, k);
        Imgproc.morphologyEx(binary, binary, Imgproc.MORPH_CLOSE, k);

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(binary, contours, new Mat(),
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area < MIN_AREA_PX || area > MAX_AREA_PX) continue;
            double per = Imgproc.arcLength(new org.opencv.core.MatOfPoint2f(c.toArray()), true);
            if (per <= 0) continue;
            double circ = 4.0 * Math.PI * area / (per * per);
            if (circ < MIN_CIRCULARITY) continue;

            Rect r = Imgproc.boundingRect(c);
            // The ball TOUCHES the floor at the bottom of its silhouette.
            double u = r.x + r.width / 2.0;
            double v = r.y + r.height;

            double[] g = groundPoint(u, v);
            if (g == null) continue;                  // ray never meets the floor

            // Predict the blob's size from the range the GEOMETRY gave us
            double diam = diameterIn(type);
            double dx = g[0] - (-C270_BEHIND_IN);
            double dy = g[1] - (-C270_RIGHT_IN);
            double dz = C270_HEIGHT_IN - diam / 2.0;
            double slant = Math.sqrt(dx * dx + dy * dy + dz * dz);
            // fx is set in init(), which the VisionPortal calls before any
            if (SIZE_CHECK && fx > 0 && slant > 1e-6) {
                double predDpx = fx * diam / slant;
                double predArea = Math.PI / 4.0 * predDpx * predDpx;
                double ratio = predArea > 0 ? area / predArea : 0;
                if (ratio < SIZE_TOL_LO || ratio > SIZE_TOL_HI) continue;
            }

            double range = Math.hypot(g[0], g[1]);
            double bearing = Math.toDegrees(Math.atan2(g[1], g[0]));
            double[] f = toField(g[0], g[1]);
            out.add(new Detection(type, f[0], f[1], range, bearing, area));
        }
    }

    public double[] groundPoint(double u, double v) {
        double xn = (u - cx) / fx;
        double yn = (v - cy) / fy;

        double p = Math.toRadians(C270_PITCH_DEG);
        // Camera basis in robot coords.
        double[] fwd   = { -Math.cos(p), 0, -Math.sin(p) };
        double[] right = { 0, 1, 0 };
        double[] down  = { Math.sin(p), 0, -Math.cos(p) };

        double rx = fwd[0] + xn * right[0] + yn * down[0];
        double ry = fwd[1] + xn * right[1] + yn * down[1];
        double rz = fwd[2] + xn * right[2] + yn * down[2];
        if (rz >= -1e-6) return null;                 // at or above the horizon

        double camX = -C270_BEHIND_IN, camY = -C270_RIGHT_IN, camZ = C270_HEIGHT_IN;
        double t = -camZ / rz;
        return new double[] { camX + t * rx, camY + t * ry };
    }

    private double[] toField(double rx, double ry) {
        if (!poseValid) return new double[] { Double.NaN, Double.NaN };
        double c = Math.cos(Math.toRadians(poseYawDeg)), s = Math.sin(Math.toRadians(poseYawDeg));
        return new double[] { poseX + rx * c - ry * s, poseY + rx * s + ry * c };
    }

    // ---- what the FSM asks for -------------------------------------------

    // Called when the processor is switched off.
    private boolean accepting = true;

    public void setAccepting(boolean on) {
        synchronized (lock) {
            accepting = on;
            if (!on) { latest = null; pollenMaskPx = 0; }
        }
    }

    public void markStale() { setAccepting(false); }

    public List<Detection> getDetections() {
        synchronized (lock) {
            return latest == null ? new ArrayList<Detection>()
                                  : new ArrayList<Detection>(latest);
        }
    }

    public int pollenInView() {
        synchronized (lock) {
            if (latest == null) return -1;
            int n = 0;
            for (Detection d : latest) if (d.type == BallType.POLLEN) n++;

            // TOUCHING BALLS.
            if (n == 0 && pollenMaskPx >= MIN_AREA_PX) return 1;
            return n;
        }
    }

    public int getPollenMaskPx() { synchronized (lock) { return pollenMaskPx; } }

    public Detection nearestBall() {
        synchronized (lock) {
            return (latest == null || latest.isEmpty()) ? null : latest.get(0);
        }
    }

    public Detection nearestWanted(int allianceNectarColorId) {
        BallType wantedNectar = allianceNectarColorId == 2
                ? BallType.NECTAR_RED : BallType.NECTAR_BLUE;
        synchronized (lock) {
            if (latest == null) return null;
            for (Detection d : latest) {
                if (d.type == BallType.POLLEN || d.type == wantedNectar) return d;
            }
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // Nothing drawn: the driver station preview costs frame rate
    }
}
