package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class ObjectFinder {
    OpenCvCamera phoneCam;
    // apply template matching
    Mat elementTemplate;
    Mat hubTemplate;
    Mat bigHubTemplate;
    Mat outputImage;
    int machMethod = Imgproc.TM_CCOEFF;
    final int width = 320;
    final int height = 240;
    final OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
    double scoreScaleV = 1.0 / width / height;

    private String imagePath = "/FIRST/images";
    private String elementTemplateFile = "/sdcard" + imagePath + "/team_element_2.png";
    private String hubTemplateFile = "/sdcard" + imagePath + "/shipping_hub.png";
    private String bigHubTemplateFile = "/sdcard" + imagePath + "/shipping_hub_close.png";


    private Object mutex;

    HighGoalPipeline highGoalPipeline;
    Point nextElementLoc;

    List<Point> ElementLocs;
    Size ElementSize;
    double ElementHalfWidth;
    double ElementHalfHeight;

    Point elementTopLeft;
    Point ElementBottomRight;
    Point centerTopLeft;
    Point centerBottomRight;
    Point objectLocation;

    Scalar ignoreColor;
    Scalar boxColor;
    Scalar ringColor;

    int mode = 0;

    PIDControl pidHeadingDeltaPower;
    PIDControl pidSideMoveDeltaPower;
    PIDControl pidForwardMoveDeltaPower;

    double targetHeading = 0.49;
    double headingTolerance = 0.015;
    int headingConvergenceCount = 0;
    boolean started = false;

    int zoneType = 0;
    double zoneConfidence = 0;
    float elementConfidenceThreshold = 0.3f;
    float hubConfidenceThreshold = 0.5f;

    int elementZone;

    int thickness = Imgproc.FILLED;

    public ObjectFinder(HardwareMap hardwareMap) {
        mutex = new Object();
        nextElementLoc = new Point(0.5, 0.5);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        objectLocation = new Point();
        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        highGoalPipeline = new HighGoalPipeline();
        phoneCam.setPipeline(highGoalPipeline);

        /**
         * Load template images
         */
        Mat readI = Imgcodecs.imread(elementTemplateFile);
        elementTemplate = new Mat();
        Imgproc.cvtColor(readI, elementTemplate, Imgproc.COLOR_RGB2BGRA);

        readI = Imgcodecs.imread(hubTemplateFile);
        hubTemplate = new Mat();
        Imgproc.cvtColor(readI, hubTemplate, Imgproc.COLOR_RGB2BGRA);

        readI = Imgcodecs.imread(bigHubTemplateFile);
        bigHubTemplate = new Mat();
        Imgproc.cvtColor(readI, bigHubTemplate, Imgproc.COLOR_RGB2BGRA);

        // internal settings
        ElementSize = new Size(elementTemplate.cols(), elementTemplate.rows());
        ElementHalfWidth = elementTemplate.cols() * 0.5;
        ElementHalfHeight = elementTemplate.rows() * 0.5;

        ignoreColor = new Scalar(0, 0, 0);
        boxColor = new Scalar(0, 0, 255, 255);
        ringColor = new Scalar(255, 0, 0, 255);
        outputImage = new Mat();
        ElementLocs = new ArrayList<>();

        elementTopLeft = new Point(0, width * 0.32);
        ElementBottomRight = new Point(height - 1, width * 0.58);
        centerTopLeft = new Point(height*0.45, width * 0.59);
        centerBottomRight = new Point(height*0.55, width * 0.7);

        pidHeadingDeltaPower = new PIDControl();
        pidHeadingDeltaPower.setKp(0.2);
        pidHeadingDeltaPower.setKi(0.8);
        pidHeadingDeltaPower.setKd(0);
        pidHeadingDeltaPower.setMaxIntegralError(0.5);

        pidSideMoveDeltaPower = new PIDControl();
        pidSideMoveDeltaPower.setKp(0.2);
        pidSideMoveDeltaPower.setKi(1.8);
        pidSideMoveDeltaPower.setKd(0);
        pidSideMoveDeltaPower.setMaxIntegralError(0.04);

        pidForwardMoveDeltaPower = new PIDControl();
        pidForwardMoveDeltaPower.setKp(0.2);
        pidForwardMoveDeltaPower.setKi(1.8);
        pidForwardMoveDeltaPower.setKd(0);
        pidForwardMoveDeltaPower.setMaxIntegralError(0.04);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //start();
                return;
            }

            @Override
            public void onError(int errorCode) {
                return;
            }
        });
    }

    public static Comparator<Point> ElementLocComparator = new Comparator<Point>() {
        public int compare(Point s1, Point s2) {
            return (int) (s1.x - s2.x);
        }
    };

    public void start() {
        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        if (!started) {
            phoneCam.startStreaming(width, height, cameraRotation);
            started = true;
        }
    }

    public void pause(){
         phoneCam.stopStreaming();
    }

    public void resume(){
        phoneCam.startStreaming(width, height, cameraRotation);
    }

    public void stop() {
        phoneCam.stopStreaming();
        started = false;
    }

    public void close() {
        phoneCam.closeCameraDevice();
        started = false;
    }

    public void setMode(int m) {
        mode = m;
    }

    public int getZoneType() {
        return zoneType;
    }

    public int autoAim(HardwareChassis chassis, double target) {
        if (started) {
            double error = targetHeading - target;
            double deltaPower
                    = Range.clip(pidHeadingDeltaPower.update(error,
                    System.currentTimeMillis() * 0.001), -1, 1);
            if (Math.abs(error) < headingTolerance) {
                headingConvergenceCount++;
            } else {
                headingConvergenceCount = 0;
            }

            if (headingConvergenceCount > 3) {
                //pidHeadingDeltaPower.reset();
                headingConvergenceCount = 0;
                //chassis.moveLeftRight(0, 0);
                chassis.moveLeftRight(-deltaPower, deltaPower);
                return 0;
            } else {
                chassis.moveLeftRight(-deltaPower, deltaPower);
                return 1;
            }
        } else {
            start();
            return 1;
        }
    }

    public double getObjectHeading(){
        synchronized (mutex) {
            return objectLocation.x/height;
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class HighGoalPipeline extends OpenCvPipeline {
        boolean viewportPaused = false;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        private void getElementLocations(Mat image, Mat input, int ElementLimit) {
            synchronized (mutex) {
                ElementLocs.clear();
                elementZone = 0;

                for (int i = 0; i < ElementLimit; i++) {
                    // pick the max,
                    Core.MinMaxLocResult r = Core.minMaxLoc(image);
                    if (r.maxVal > elementConfidenceThreshold) {
                        zoneConfidence = r.maxVal;
                        objectLocation.x = r.maxLoc.x+elementTemplate.width()*0.5;
                        objectLocation.y = r.maxLoc.y+elementTemplate.height()*0.5;

                        ElementLocs.add(new Point(r.maxLoc.x + elementTopLeft.x, r.maxLoc.y + elementTopLeft.y));

                        // fill Element
                        Imgproc.rectangle(image,
                                new Point(r.maxLoc.x - ElementHalfWidth, r.maxLoc.y - ElementHalfHeight),
                                new Point(r.maxLoc.x + ElementHalfWidth, r.maxLoc.y + ElementHalfHeight),
                                ignoreColor, thickness);

                        // label element
                        double element = r.maxLoc.x + elementTemplate.width() * 0.5;
                        if (element <= 80) {
                            elementZone = 1;
                            Imgproc.putText(input, "A", elementTopLeft,
                                    Imgproc.FONT_HERSHEY_SIMPLEX, 1, ringColor, 3);

                        } else if (element <= 160 && element >= 80) {
                            elementZone = 2;
                            Imgproc.putText(input, "B", elementTopLeft,
                                    Imgproc.FONT_HERSHEY_SIMPLEX, 1, ringColor, 3);
                        } else {
                            elementZone = 3;
                            Imgproc.putText(input, "C", elementTopLeft,
                                    Imgproc.FONT_HERSHEY_SIMPLEX, 1, ringColor, 3);
                        }
                        break;
                    }
                }

                // sort location left to right
                //Collections.sort(ElementLocs, ObjectFinder.ElementLocComparator);
            }

            // Element location
            for (Point ElementLoc : ElementLocs) {
                Imgproc.rectangle(input, ElementLoc,
                        new Point(ElementLoc.x + elementTemplate.cols(),
                                ElementLoc.y + elementTemplate.rows()),
                        ringColor, 2);


            }

        }

        private void getHubLocations(Mat image, Mat input) {
            synchronized (mutex) {
                // pick the max,
                Core.MinMaxLocResult r = Core.minMaxLoc(image);
                if (r.maxVal > hubConfidenceThreshold) {
                    zoneConfidence = r.maxVal;
                    if (mode == 1) {
                        objectLocation.x = r.maxLoc.x + hubTemplate.width() * 0.5;
                        objectLocation.y = r.maxLoc.y + hubTemplate.height() * 0.5;
                        Imgproc.rectangle(input, r.maxLoc,
                                new Point(r.maxLoc.x + hubTemplate.cols(),
                                        r.maxLoc.y + hubTemplate.rows()),
                                ringColor, 2);
                    } else {
                        objectLocation.x = r.maxLoc.x + bigHubTemplate.width() * 0.5;
                        objectLocation.y = r.maxLoc.y + bigHubTemplate.height() * 0.5;
                        Imgproc.rectangle(input, r.maxLoc,
                                new Point(r.maxLoc.x + bigHubTemplate.cols(),
                                        r.maxLoc.y + bigHubTemplate.rows()),
                                ringColor, 2);
                    }

                    // save hub location
                }
            }

        }

        @Override
        public Mat processFrame(Mat input) {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            switch(mode) {
                case 0: {
                    Imgproc.rectangle(input, elementTopLeft, ElementBottomRight, boxColor, 2);
                    Imgproc.rectangle(input, centerTopLeft, centerBottomRight, boxColor, 3);
                    // find team elements
                    Mat subImg = input.submat((int) elementTopLeft.y, (int) ElementBottomRight.y,
                            (int) elementTopLeft.x, (int) ElementBottomRight.x);
                    Imgproc.matchTemplate(subImg, elementTemplate, outputImage, Imgproc.TM_CCOEFF_NORMED);

                    getElementLocations(outputImage, input, 3);
                }
                break;
                case 1: {
                    // find hubs
                    Imgproc.matchTemplate(input, hubTemplate, outputImage, Imgproc.TM_CCOEFF_NORMED);
                    getHubLocations(outputImage, input);
                }
                break;
                case 2: {
                    // find big hubs
                    Imgproc.matchTemplate(input, bigHubTemplate, outputImage, Imgproc.TM_CCOEFF_NORMED);
                    getHubLocations(outputImage, input);
                }
                break;
                default:
            }

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped() {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                phoneCam.pauseViewport();
            } else {
                phoneCam.resumeViewport();
            }
        }
    }
}
