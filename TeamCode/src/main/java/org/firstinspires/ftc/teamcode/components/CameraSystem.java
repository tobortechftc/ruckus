package org.firstinspires.ftc.teamcode.components;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.support.CoreSystem;
//import org.firstinspires.ftc.teamcode.hardware17.SwerveUtilLOP;
//import org.firstinspires.ftc.teamcode.hardware17.TaintedAccess;

/**
 * Put brief class description here...
 */
public class CameraSystem {
    public boolean use_verbose = false;
    public boolean use_Vuforia = true;
    public boolean use_camera = false;
    public boolean use_OpenCV = true;

    public Bitmap bitmap = null;
    public boolean camReady = false;

    WebcamName webcamName;

    public VuforiaLocalizer vuforia;

    public VuforiaTrackables relicTrackables;
    public VuforiaTrackable relicTemplate;
    VuforiaLocalizer.Parameters parameters;
    private String lastError = "";

    // Central core of robot
    CoreSystem core;
    ElapsedTime runtime;

    public CameraSystem(CoreSystem core) {
        this.core = core;
    }

    public void enable(boolean isAuto) {
        if (isAuto) {
            use_Vuforia = true;
            use_camera = true;
            use_OpenCV = true;
        } else {
            use_Vuforia = false;
            use_camera = false;
            use_OpenCV = false;
        }
    }

    public void disable() {
        use_Vuforia = false;
        use_camera = false;
    }

    public void init(HardwareMap hwMap) {

        if (use_Vuforia) {
//            webcamName = hwMap.get(WebcamName.class, "Webcam 1");
            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            parameters.vuforiaLicenseKey = "AaaZDWL/////AAAAGYIaD+Gn/UUDhEiR/gcOJxdEJlKEpCOKSLPfhYJfYthUNZ0vnEGm0VGPutkNgRq8bq1ufm3eAySnLhkJQ7d4w6VDT7os5FGPEOGPfsIWMYNAFMdX+wlJo2JCyljeSxQtXUd/YileyfYKBXOl2uFA4KnStCC9WkYTUBrAof3H7RGKorzYixDeOpbmCsf25rayjtAUQrKCwG4j6P5rRdxy7SC//v4VC6NirNwgJ/xn1r02/jbx8vUDrDODGyut9iLk06IzMnrq/P01yKOp48clTw0WIKNmVT7WUQweXy+E1w6xwFplTlPkjC+gzerDOpxHPqYg8RusWD2Y/IMlmnk1yzJba1B9Xf9Ih6BJbm/fVwL4";

            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//            parameters.cameraName = webcamName;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            vuforia.enableConvertFrameToBitmap();
            relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
            relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        }

        if (use_camera) {
            if (!use_Vuforia) {
                throw new IllegalStateException("use_camera cannot be flagged as true without use_Vuforia also being true!");
            }
        }
    }

    CameraSystem(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

//    int getColumnIndex(RelicRecoveryVuMark vuMark) throws InterruptedException {
//        // return row index for Cryptograph
//        // unknown : -1
//        // left    :  0
//        // center  :  1
//        // right   :  2
//        if (vuMark == RelicRecoveryVuMark.LEFT)
//            return 0;
//        else if (vuMark == RelicRecoveryVuMark.CENTER)
//            return 1;
//        else if (vuMark == RelicRecoveryVuMark.RIGHT)
//            return 2;
//
//        return -1;
//    }

//    public int get_cryptobox_column() throws InterruptedException {
//
//        int column = -1;
//        if (!use_Vuforia)
//            return column;
//
//        relicTrackables.activate();
//        runtime.reset();
//        while (runtime.seconds() < 2.0 && column == -1) {
//            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
//            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//
//                /* Found an instance of the template. In the actual game, you will probably
//                 * loop until this condition occurs, then move on to act accordingly depending
//                 * on which VuMark was visible. */
//                column = getColumnIndex(vuMark);
//            }
//        }
//        return column;
//    }

    /**
     * Opposite of stopCamera(), it sets the Vuforia frame queue capacity to one and sets the color format of the frames to allow it to be processed
     *
     * @author Mason Mann
     */
    public void initVuforia() {
        this.vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

//    String getLastError() {
//        return lastError;
//    }

    /**
     * Takes in a Vuforia CloseableFrame and converts it to a Bitmap variable to be processed
     *
     * @param frame
     * @return
     * @author Mason Mann
     */
    private Bitmap convertFrameToBitmap(VuforiaLocalizer.CloseableFrame frame) {
        long numImages = frame.getNumImages();
        Image image = null;

        for (int imageI = 0; imageI < numImages; imageI++) {
            if (frame.getImage(imageI).getFormat() == PIXEL_FORMAT.RGB565) {
                image = frame.getImage(imageI);
                break;
            }
        }
        if (image == null) {
            for (int imageI = 0; imageI < numImages; imageI++) {
                //For diagnostic purposes
            }
            lastError = "Failed to get RGB565 image format!";
            return null;
        }
        Bitmap bitmap = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        bitmap.copyPixelsFromBuffer(image.getPixels());
        return bitmap;
    }

    private Bitmap cropBitmap(Bitmap source, double xOffsetF, double yOffsetF, double widthF, double heightF) {
        int offset_x = (int) (source.getWidth() * xOffsetF);
        int offset_y = (int) (source.getHeight() * yOffsetF);
        int width = (int) (source.getWidth() * widthF);
        int height = (int) (source.getHeight() * heightF);
        Bitmap destBitmap = Bitmap.createBitmap(source, offset_x, offset_y, width, height);
        return destBitmap;
    }

    enum CameraSelect {
        PHONE,
    }

    /**
     * @return Bitmap
     * @author Mason Mann
     */
    public Bitmap captureVuforiaBitmap(/*double xOffsetF, double yOffsetF, double widthF, double heightF*/) {
        Bitmap bitmapTemp = null;
        lastError = "";
        int capacity = vuforia.getFrameQueueCapacity();
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().poll();

        if (frame == null) {
            lastError = "ERROR! Failed to retrieve frame!";
            return null;
        }

        bitmapTemp = convertFrameToBitmap(frame);

        frame.close();
        if (bitmapTemp == null) {
            lastError = "ERROR! Failed to retrieve bitmap";
            return null;

        }
        //White Balance applied here
//        int whitestPixel = getWhitestPixel(bitmapTemp);
//        applyWhiteBalance(bitmapTemp, whitestPixel);

        //Bitmap bitmap = cropBitmap(bitmapTemp, xOffsetF, yOffsetF, widthF, heightF);
        Bitmap bitmap = bitmapTemp;
        return bitmap;
    }

    /**
     * @param source
     * @return
     * @author Mason Mann
     * Takes a Bitmap as input and outputs an integer of the color constant of the "whitest" pixel
     */
    int getWhitestPixel(Bitmap source) {
        int[] pixels = new int[source.getHeight() * source.getWidth()];

        int whitestPixel = 0;
        int whitestPixelColorAvg = 0;

        source.getPixels(pixels, 0, source.getWidth(), 0, 0, source.getWidth(), source.getHeight());

        for (int pixeli = 0; pixeli < pixels.length; pixeli++) {

            int pixelRed = Color.red(pixels[pixeli]);
            int pixelGreen = Color.green(pixels[pixeli]);
            int pixelBlue = Color.blue(pixels[pixeli]);

            int pixeliColorAvg = (pixelRed + pixelGreen + pixelBlue) / 3;

            if (pixeliColorAvg > whitestPixelColorAvg) {
                whitestPixel = pixeliColorAvg;
                whitestPixelColorAvg = pixeliColorAvg;
            }
        }
        if (whitestPixel / Color.WHITE > .8) {
            return whitestPixel;
        } else {
            whitestPixel = 2;
            return whitestPixel;
        }
    }

    /**
     * @param source
     * @param whitestPixel
     * @return
     * @author Mason Mann
     * Takes returned integer from getWhitestPixel and a source bitmap then returns a white balanced bitmap
     */
    Bitmap applyWhiteBalance(Bitmap source, int whitestPixel) {
        if (whitestPixel == 2) {
            return source;
        } else {
            if (Color.red(whitestPixel) != 0 && Color.green(whitestPixel) != 0 && Color.red(whitestPixel) != 0) {
                double rComp = 255.0 / Color.red(whitestPixel);
                double gComp = 255.0 / Color.green(whitestPixel);
                double bComp = 255.0 / Color.blue(whitestPixel);

                int w = source.getWidth();
                int h = source.getHeight();

                for (int i = 0; i < w; i++) {
                    for (int j = 0; j < h; j++) {
                        int pixColor = source.getPixel(i, j);

                        int inR = Color.red(pixColor);
                        int inG = Color.green(pixColor);
                        int inB = Color.blue(pixColor);

                        double rDoub = inR * rComp;
                        double gDoub = inG * gComp;
                        double bDoub = inB * bComp;

                        source.setPixel(i, j, Color.argb(255, (int) rDoub, (int) gDoub, (int) bDoub));

                        if (source.getConfig() != Bitmap.Config.RGB_565) {
                            source.setConfig(Bitmap.Config.RGB_565);
                        }
                    }
                }
            }
            return source;
        }
    }

    /**
     * Opposite of initVuforia(), sets frame queue capacity to zero and disables the frame format set by initVuforia()
     *
     * @author Mason Mann
     */
    public void stopCamera() {
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, false);
        this.vuforia.setFrameQueueCapacity(0);
    }

    public void show_telemetry(Telemetry telemetry) {
        telemetry.addData("VuMark", "%s visible", RelicRecoveryVuMark.from(relicTemplate));
    }

}
