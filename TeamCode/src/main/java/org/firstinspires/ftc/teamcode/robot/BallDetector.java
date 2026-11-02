package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * C270 ball finder: yellow POLLEN, red and blue NECTAR on the tiles.
 *
 * Two jobs:
 *   1. Count POLLEN in front of the robot, which is how INTAKE_FLOWER knows
 *      when a flower is empty.
 *   2. Give the nearest ball as a FIELD position so PedroPathing can drive to
 *      it, in autonomous or on a driver button.
 *
 * GEOMETRY
 *   The camera is rear-facing, fixed, 40 deg below horizontal, 308.175 mm up,
 *   149.128 mm behind centre and 44 mm right. Balls sit ON the tiles, so a
 *   detection's ground contact point is recoverable: project the bottom of the
 *   blob onto the z = 0 plane. That is what makes range and bearing possible
 *   from one camera with no stereo and no known ball size.
 *
 *   Visible strip is roughly 15 to 31 in behind the robot centre. A ball
 *   closer than that leaves the frame -- by design, because the intake swings
 *   out to meet it there.
 *
 * CALIBRATION
 *   The HSV ranges below are STARTING POINTS. They are in OpenCV units, where
 *   HUE IS 0-179, not 0-360 -- the values measured with your REV colour
 *   sensors are on the 0-360 scale and must be halved before they mean
 *   anything here. Lighting changes them too. Tune with the HSV page in
 *   HardwareCheck before trusting any of it.
 */
public class BallDetector implements VisionProcessor {

    public enum BallType { POLLEN, NECTAR_RED, NECTAR_BLUE }

    /** One ball, in the field frame Pedro uses. */
    public static final class Detection {
        public final BallType type;
        /** Ground contact point, PEDRO inches. */
        public final double fieldX, fieldY;
        /** Range and bearing from the robot centre, inches / degrees CCW. */
        public final double rangeIn, bearingDeg;
        /** Blob area in pixels -- bigger is nearer and more trustworthy. */
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

    /** Reject specks and reject the whole frame going one colour. */
    public static double MIN_AREA_PX = 300;
    public static double MAX_AREA_PX = 90000;
    /** 1.0 is a perfect circle. Balls are round; floor tape and shadows are not. */
    public static double MIN_CIRCULARITY = 0.55;

    // ---- camera, from RobotHardware --------------------------------------
    public static double CAM_HEIGHT_IN = RobotHardware.C270_HEIGHT_IN;
    public static double CAM_BEHIND_IN = RobotHardware.C270_BEHIND_IN;
    public static double CAM_RIGHT_IN  = RobotHardware.C270_RIGHT_IN;
    /**
     * MAGNITUDE of the downward pitch, positive. RobotHardware states the same
     * angle as -40 because it uses the SDK's convention, where negative is
     * below horizontal; the basis vectors in groundPoint() are built for the
     * positive form, so the sign is flipped here rather than hard-coded.
     *
     * It used to be a bare 40.0 -- the only camera constant not derived from
     * RobotHardware, so re-mounting the camera there would have left every
     * range quietly wrong. Same value today, but it tracks now.
     */
    public static double CAM_PITCH_DEG = -RobotHardware.C270_PITCH_DEG;

    /**
     * Logitech's published figure for the C270, and the only FOV number worth
     * trusting: 55 deg DIAGONAL.
     *
     * Horizontal and vertical both fall out of it once the capture aspect is
     * known, so deriving them here means the code stays correct whatever
     * resolution the portal ends up using:
     *      640x480  (4:3)   -> 45.2 x 34.7
     *      1280x720 (16:9)  -> 48.8 x 28.6
     * The portal uses the FIRST pair -- 640x480. The taller 4:3 frame sees a
     * 21 in deep strip of tiles against 16 in at 720p, for a quarter of the
     * pixels. Nothing here is hard-coded to that: the focal length comes from
     * the live frame size, so changing the portal's resolution is safe.
     */
    public static double CAM_DIAGONAL_FOV_DEG = RobotHardware.C270_DIAGONAL_FOV_DEG;

    private int imgW = 640, imgH = 480;
    private double fx, fy, cx, cy;

    // robot pose, pushed in each loop so detections can be reported in field
    private volatile double poseX = 72, poseY = 72, poseYawDeg = 0;
    private volatile boolean poseValid = false;

    private final Object lock = new Object();
    private List<Detection> latest = new ArrayList<Detection>();
    /** Pollen-coloured pixels in the last frame, before shape filtering. */
    private int pollenMaskPx = 0;

    // scratch, reused so the processor does not allocate every frame
    private final Mat hsv = new Mat(), m1 = new Mat(), m2 = new Mat(), mask = new Mat();

    /** Mean HSV over a small patch at frame centre. The fastest way to tune:
     *  hold a ball in the middle of the view and read what it actually is,
     *  instead of guessing gates and seeing what sticks. */
    private volatile double centreH = 0, centreS = 0, centreV = 0;
    public static int CENTRE_PATCH_PX = 40;

    public double[] getCentreHsv() { return new double[] { centreH, centreS, centreV }; }

    /**
     * Mean of the SAME patch in the frame as delivered, before any colour
     * conversion. This is the only way to tell a bad gate from a bad channel
     * order: on a yellow ball the red and green channels must both be high and
     * blue low. If channel 0 comes back LOW and channel 2 HIGH, the frame is
     * BGR and we are converting it as if it were RGB.
     *
     *     yellow ball, roughly RGB(230,200,40)
     *         converted correctly        -> OpenCV H ~ 25
     *         converted with R/B swapped -> OpenCV H ~ 95
     *     red nectar     correct H ~ 0     swapped H ~ 120
     *     blue nectar    correct H ~ 116   swapped H ~ 4
     *
     * A first reading of H ~ 100 on a yellow ball matches the swapped column
     * almost exactly, which is why this readout exists.
     */
    private volatile double centre0 = 0, centre1 = 0, centre2 = 0;

    public double[] getCentreRaw() { return new double[] { centre0, centre1, centre2 }; }

    /**
     * Channel order of the frames the VisionPortal hands us. Flip this if the
     * raw readout above shows the swap, then re-measure the gates -- do not
     * try to compensate by moving the hue windows, because red and blue trade
     * places and the two nectar colours would end up detecting each other.
     */
    public static boolean FRAME_IS_BGR = false;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        imgW = width; imgH = height;
        computeIntrinsics();
    }

    /**
     * One focal length from the published diagonal FOV. Square pixels, so
     * fx == fy; the horizontal and vertical FOVs are consequences of that and
     * of the frame's aspect, not independent inputs.
     *
     * Still worth checking with a tape at a known distance before trusting
     * ranges -- an error here scales every range estimate linearly.
     */
    private void computeIntrinsics() {
        double diagPx = Math.hypot(imgW, imgH);
        double f = (diagPx / 2.0) / Math.tan(Math.toRadians(CAM_DIAGONAL_FOV_DEG / 2.0));
        fx = f; fy = f;
        cx = imgW / 2.0;
        cy = imgH / 2.0;
    }

    /** Horizontal FOV actually in effect, degrees. Telemetry only. */
    public double hfovDeg() { return 2 * Math.toDegrees(Math.atan((imgW / 2.0) / fx)); }
    /** Vertical FOV actually in effect, degrees. Telemetry only. */
    public double vfovDeg() { return 2 * Math.toDegrees(Math.atan((imgH / 2.0) / fy)); }

    /** Call every loop from the OpMode with the Pedro pose. */
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
        synchronized (lock) { latest = found; pollenMaskPx = pollenPx; }
        return null;
    }

    /**
     * Mean of a centre patch, for threshold tuning -- in HSV, and in the raw
     * frame alongside it so a channel-order fault is visible rather than
     * showing up as hues that refuse to make sense.
     */
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

    /** Live gate editing, for the tuning page. */
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

            double range = Math.hypot(g[0], g[1]);
            double bearing = Math.toDegrees(Math.atan2(g[1], g[0]));
            double[] f = toField(g[0], g[1]);
            out.add(new Detection(type, f[0], f[1], range, bearing, area));
        }
    }

    /**
     * Pixel -> ground contact point in the ROBOT frame (+X forward, +Y left),
     * inches. Null when the ray points at or above the horizon.
     */
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

    /** Robot-frame point -> PEDRO field inches, using the last pose given. */
    private double[] toField(double rx, double ry) {
        if (!poseValid) return new double[] { Double.NaN, Double.NaN };
        double c = Math.cos(Math.toRadians(poseYawDeg)), s = Math.sin(Math.toRadians(poseYawDeg));
        return new double[] { poseX + rx * c - ry * s, poseY + rx * s + ry * c };
    }

    // ---- what the FSM asks for -------------------------------------------

    public List<Detection> getDetections() {
        synchronized (lock) { return new ArrayList<Detection>(latest); }
    }

    /**
     * POLLEN currently in view. While the robot is at a flower this IS the
     * flower's count -- balls sit on the tiles there too, so position rather
     * than height is what separates flower pollen from field pollen, and
     * being at the flower supplies that context.
     *
     * Returns -1 only if no frame has been processed, which the FSM treats as
     * VISION_UNAVAILABLE rather than "empty".
     */
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

    /** Pollen-coloured pixels in the last frame, before any shape filtering. */
    public int getPollenMaskPx() { synchronized (lock) { return pollenMaskPx; } }

    /** Nearest ball of any type, or null. Already sorted by range. */
    public Detection nearestBall() {
        synchronized (lock) { return latest.isEmpty() ? null : latest.get(0); }
    }

    /** Nearest ball we would actually want to collect. */
    public Detection nearestWanted(int allianceNectarColorId) {
        BallType wantedNectar = allianceNectarColorId == 2
                ? BallType.NECTAR_RED : BallType.NECTAR_BLUE;
        synchronized (lock) {
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




