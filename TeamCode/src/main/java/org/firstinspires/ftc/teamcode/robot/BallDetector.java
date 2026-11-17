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
    //
    // MEASURED on the bench, C270 at 640x480, each ball filling the centre
    // patch. Channel order confirmed RGB at the same time -- yellow read
    // ch0/ch1 high and ch2 low, which is what rules out a swap.
    //
    //              H     S     V        gate set from it
    //   pollen     29   155   161       H 21-37   S>=90   V>=95
    //   red        173  138   209       H 163-10  S>=80   V>=100
    //   blue       106  129   255       H 96-118  S>=75   V>=80
    //
    // Hue windows are the measurement +/-8. The S and V floors are well under
    // 0.6x what was measured, because the bench is better lit than a gym and
    // under-saturating is how you lose a ball you can see.
    public static Scalar YELLOW_LO = new Scalar(21, 90, 95);
    public static Scalar YELLOW_HI = new Scalar(37, 255, 255);
    // Red wraps around 0, so it needs two windows. The ball measured 173, only
    // 3 off the old window's 170 edge -- a little shadow and it fell out of
    // the gate entirely. 163 gives it room on the side it actually sits.
    public static Scalar RED1_LO = new Scalar(0, 80, 100);
    public static Scalar RED1_HI = new Scalar(10, 255, 255);
    public static Scalar RED2_LO = new Scalar(163, 80, 100);
    public static Scalar RED2_HI = new Scalar(179, 255, 255);
    // Blue measured V 255 -- clipped, the camera is blowing this ball out. Its
    // S 129 cleared the old 120 floor by 9 counts, which is not margin, it is
    // luck. V floor stays low because a shadowed blue ball is the real risk.
    public static Scalar BLUE_LO = new Scalar(96, 75, 80);
    public static Scalar BLUE_HI = new Scalar(118, 255, 255);

    // Ball diameters, MEASURED 14 Nov. The two types are NOT the same size,
    // and the difference is large: area scales with the square, so at equal
    // range a nectar blob is 1.65x a pollen one.
    public static double POLLEN_DIAM_IN = 2.8;
    public static double NECTAR_DIAM_IN = 3.6;

    // Cheap pre-filter only. Run the numbers for this camera (768 px focal,
    // 12.13 in high, pitched 40 deg down) and the ground is visible from 7.8
    // to 29.1 in in front of the lens -- a narrow band, and within it a real
    // ball covers:
    //
    //     pollen   3,786 .. 20,697 px2
    //     nectar   6,314 .. 35,937 px2
    //
    // The old gates were 300 and 90000: twelve times too low and two and a
    // half times too high. They admitted anything and rejected nothing, so a
    // yellow scuff on the tile passed every test and became a Detection that
    // BallSeek would drive at. These are speckle/merge bounds now; the real
    // work is the size check below.
    public static double MIN_AREA_PX = 1200;
    public static double MAX_AREA_PX = 60000;
    public static double MIN_CIRCULARITY = 0.55;

    // Size consistency. Range comes from the ground-plane projection, which
    // never looks at the blob's size, so predicting the area from that range
    // and comparing is a genuinely INDEPENDENT test -- not circular. A floor
    // mark projects to some range perfectly happily and then turns out to be
    // the wrong size for a ball at that range, which is how it gets caught.
    //
    // Tolerances are deliberately loose. Balls get clipped at frame edges and
    // the morphology erodes blobs, so under-reading is normal; over-reading
    // means two balls have merged. This is here to reject things that are not
    // balls, not to grade the ones that are.
    // TURN THIS OFF TO TEST BY HAND. The check works by predicting the area
    // from the range, and the range comes from assuming the ball TOUCHES THE
    // FLOOR. Hold a ball up in front of the camera and that assumption is
    // false: the projection puts it much further away than it is, the
    // predicted area comes out far too small, and the detection is thrown
    // away. The detector then reports nothing for a ball plainly in view,
    // which looks like a broken detector rather than a violated assumption.
    // Step 14 has A bound to this for exactly that reason.
    public static boolean SIZE_CHECK = true;
    public static double SIZE_TOL_LO = 0.35;
    public static double SIZE_TOL_HI = 2.50;

    public static double diameterIn(BallType t) {
        return t == BallType.POLLEN ? POLLEN_DIAM_IN : NECTAR_DIAM_IN;
    }

    // What a ball of this type SHOULD cover at this range. Exposed so step 14
    // can show predicted against measured: if a real ball sits consistently
    // outside the tolerance, the tolerance is wrong, and that is only visible
    // if the number is on the screen.
    public double expectedAreaPx(BallType t, double rangeIn, double bearingDeg) {
        double diam = diameterIn(t);
        double g0 = rangeIn * Math.cos(Math.toRadians(bearingDeg));
        double g1 = rangeIn * Math.sin(Math.toRadians(bearingDeg));
        double dx = g0 + CAM_BEHIND_IN, dy = g1 + CAM_RIGHT_IN;
        double dz = CAM_HEIGHT_IN - diam / 2.0;
        double slant = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (slant < 1e-6) return 0;
        double d = fx * diam / slant;
        return Math.PI / 4.0 * d * d;
    }

    // ---- camera, from RobotHardware --------------------------------------
    public static double CAM_HEIGHT_IN = RobotHardware.C270_HEIGHT_IN;
    public static double CAM_BEHIND_IN = RobotHardware.C270_BEHIND_IN;
    public static double CAM_RIGHT_IN  = RobotHardware.C270_RIGHT_IN;
    public static double CAM_PITCH_DEG = -RobotHardware.C270_PITCH_DEG;

    public static double CAM_DIAGONAL_FOV_DEG = RobotHardware.C270_DIAGONAL_FOV_DEG;

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
        double f = (diagPx / 2.0) / Math.tan(Math.toRadians(CAM_DIAGONAL_FOV_DEG / 2.0));
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
        // Count the pixels BEFORE collect() applies its shape filter, so
        // pollenInView() can tell "no pollen" from "pollen the round-blob
        // filter rejected because two balls were touching".
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
        // Only while accepting. setProcessorEnabled(false) does not stop a
        // frame already in flight, so without this test that frame completes
        // after markStale() and writes detections back -- resurrecting a
        // previous flower's count while the detector is supposed to be off.
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
        // are colour, and Core.mean fills the rest with zero regardless.
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
                // window anchors at 179; only the width and the S/V floors move.
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
        // ball, which otherwise fragments into several small blobs.
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
            // Using the centroid would place it half a ball-diameter too far.
            double u = r.x + r.width / 2.0;
            double v = r.y + r.height;

            double[] g = groundPoint(u, v);
            if (g == null) continue;                  // ray never meets the floor

            // Predict the blob's size from the range the GEOMETRY gave us and
            // compare with what was actually measured. Distance is to the
            // ball's CENTRE, half a diameter off the floor, not to the
            // contact point -- using the contact point would over-predict the
            // area for the near balls, which are the ones that matter most.
            double diam = diameterIn(type);
            double dx = g[0] - (-CAM_BEHIND_IN);
            double dy = g[1] - (-CAM_RIGHT_IN);
            double dz = CAM_HEIGHT_IN - diam / 2.0;
            double slant = Math.sqrt(dx * dx + dy * dy + dz * dz);
            // fx is set in init(), which the VisionPortal calls before any
            // frame -- but if that ever changed, predArea would be 0, every
            // ratio would be 0, and the check would silently reject EVERY
            // detection. Fail open: a check that cannot run must not be the
            // thing that empties the list.
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

        double p = Math.toRadians(CAM_PITCH_DEG);
        // Camera basis in robot coords. Rear-facing, so its "right" is the
        // robot's LEFT -- looking backwards reverses the sense.
        double[] fwd   = { -Math.cos(p), 0, -Math.sin(p) };
        double[] right = { 0, 1, 0 };
        double[] down  = { Math.sin(p), 0, -Math.cos(p) };

        double rx = fwd[0] + xn * right[0] + yn * down[0];
        double ry = fwd[1] + xn * right[1] + yn * down[1];
        double rz = fwd[2] + xn * right[2] + yn * down[2];
        if (rz >= -1e-6) return null;                 // at or above the horizon

        double camX = -CAM_BEHIND_IN, camY = -CAM_RIGHT_IN, camZ = CAM_HEIGHT_IN;
        double t = -camZ / rz;
        return new double[] { camX + t * rx, camY + t * ry };
    }

    private double[] toField(double rx, double ry) {
        if (!poseValid) return new double[] { Double.NaN, Double.NaN };
        double c = Math.cos(Math.toRadians(poseYawDeg)), s = Math.sin(Math.toRadians(poseYawDeg));
        return new double[] { poseX + rx * c - ry * s, poseY + rx * s + ry * c };
    }

    // ---- what the FSM asks for -------------------------------------------

    // Called when the processor is switched off. Results outlive the frames
    // that produced them, and a count from the LAST flower read as current
    // is the one error that hurts -- a stale 0 abandons a flower that is
    // still full. Null makes pollenInView() report -1 instead, which the FSM
    // treats as "camera unavailable, keep collecting".
    //
    // This also revives a branch that was dead: latest was initialised to an
    // empty list and never set null, so the -1 path could not happen and the
    // documented blind fallback was unreachable.
    // Gate on results, not just a one-shot clear. Set false BEFORE disabling
    // the processor so any frame mid-flight is discarded rather than landing
    // after the clear.
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

            // TOUCHING BALLS. Two balls in contact merge into one peanut-shaped
            // contour whose circularity is well under MIN_CIRCULARITY, so the
            // round-blob filter throws it away and n comes back 0 -- with a
            // clearly visible pile of pollen in frame.
            //
            // That is the one error that actually hurts: INTAKE_FLOWER_* reads
            // this ONLY as "is there any pollen left", and a false 0 makes it
            // quit a flower that still has balls in it. Undercounting 3 as 2
            // costs nothing, because the number itself is never used.
            //
            // So when the shape filter finds nothing but a ball's worth of
            // pollen-coloured pixels is present, report 1. Pessimistic about
            // the count, honest about the only question being asked.
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
        // Nothing drawn: the driver station preview costs frame rate and this
        // camera is not something a driver aims by eye.
    }
}
