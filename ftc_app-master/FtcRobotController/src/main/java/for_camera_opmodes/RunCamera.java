package for_camera_opmodes;

import android.graphics.Bitmap;
import android.graphics.YuvImage;
import android.hardware.Camera;
import android.hardware.Camera.CameraInfo;
import android.hardware.Camera.Parameters;
import android.hardware.Camera.PreviewCallback;
import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

//TODO : What the heck does some of this code do

public class RunCamera extends LinearOpMode {
    public Camera camera;
    public CameraPreview preview; //displays image from camera on phone. For some reason we need this.
    public int imageWidth;
    public int imageHeight;
    public YuvImage yuvImage = null; //image in yuv color format
    private volatile boolean imageReady = false;
    private int loopCount = 0;
    private String data;
    private int downsampling = 1;

    //Prepares the image preview so we can use the image
    public PreviewCallback previewCallback = new PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            try {
                Parameters parameters = camera.getParameters();
                RunCamera.this.imageWidth = parameters.getPreviewSize().width;
                RunCamera.this.imageHeight = parameters.getPreviewSize().height;
                RunCamera.this.yuvImage = new YuvImage(data, 17, RunCamera.this.imageWidth, RunCamera.this.imageHeight, (int[])null);
                RunCamera.this.imageReady = true;
                RunCamera.this.loopCount = RunCamera.this.loopCount + 1;
            } catch (Exception var4) {
                ;
            }
        }
    };

    public RunCamera() {
    }

    public void runOpMode() throws InterruptedException {
    }

    public void setCameraDownsampling(int downSampling) {
        this.downsampling = downSampling;
    }

    public boolean imageReady() {
        return this.imageReady;
    }

    public boolean isCameraAvailable() {
        int cameraId = -1; // -1 is front camera (not selfie camera)
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();

        for(int i = 0; i < numberOfCameras; ++i) {
            CameraInfo info = new CameraInfo();
            Camera.getCameraInfo(i, info);
            if(info.facing == 0) {
                cameraId = i;
                break;
            }
        }

        try {
            cam = Camera.open(cameraId);
        } catch (Exception var6) {
            Log.e("Error", "Camera Not Available!");
            return false;
        }

        cam.release();
        cam = null;
        return true;
    }

    public Camera openCamera(int cameraInfoType) {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();

        for(int e = 0; e < numberOfCameras; ++e) {
            CameraInfo info = new CameraInfo();
            Camera.getCameraInfo(e, info);
            if(info.facing == cameraInfoType) {
                cameraId = e;
                break;
            }
        }

        try {
            cam = Camera.open(cameraId);
        } catch (Exception var7) {
            Log.e("Error", "Can\'t Open Camera");
        }

        return cam;
    }

    public void startCamera() {
        this.camera = this.openCamera(0);
        this.camera.setPreviewCallback(this.previewCallback);
        Parameters parameters = this.camera.getParameters();
        this.imageWidth = parameters.getPreviewSize().width / this.downsampling;
        this.imageHeight = parameters.getPreviewSize().height / this.downsampling;
        parameters.setPreviewSize(this.imageWidth, this.imageHeight);
        this.camera.setParameters(parameters);
        this.data = parameters.flatten();
        if(this.preview == null) {
            ((FtcRobotControllerActivity)this.hardwareMap.appContext).initPreviewLinear(this.camera, this, this.previewCallback);
        }

    }

    public void stopCameraInSecs(int duration) {
        Thread cameraKillThread = new Thread(new CameraKillThread(duration));
        cameraKillThread.start();
    }

    public void stopCamera() {
        if(this.camera != null) {
            if(this.preview != null) {
                ((FtcRobotControllerActivity)this.hardwareMap.appContext).removePreviewLinear(this);
                this.preview = null;
            }

            this.camera.stopPreview();
            this.camera.setPreviewCallback((PreviewCallback)null);
            if(this.camera != null) {
                this.camera.release();
            }

            this.camera = null;
        }

    }

    public static int red(int pixel) {
        return pixel >> 16 & 255;
    }

    public static int green(int pixel) {
        return pixel >> 8 & 255;
    }

    public static int blue(int pixel) {
        return pixel & 255;
    }

    public static int gray(int pixel) {
        return red(pixel) + green(pixel) + blue(pixel);
    }

    public static Bitmap convertYuvImageToRgb(YuvImage yuvImage, int width, int height, int downSample) {
        return OpModeCamera.convertYuvImageToRgb(yuvImage, width, height, downSample);
    }

    public class CameraKillThread implements Runnable {
        int dur;

        public CameraKillThread(int duration) {
            this.dur = duration;
        }

        public void run() {
            try {
                Thread.sleep((long)(this.dur * 1000), 0);
            } catch (InterruptedException var2) {
                ;
            }

            RunCamera.this.stopCamera();
            RunCamera.this.imageReady = false;
        }
    }

    //Determines whether majority of pixels are blue or red, returns a color string
    public String RedOrBlue(){

        String colorString = "ERROR";
        Bitmap SavedBitmap = null;

        if (imageReady()) { // Only do this if an image has been returned from the camera
            int redValue = 0;
            int blueValue = 0;

            // Get image, rotated so (0,0) is in the bottom left of the preview window
            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, imageWidth, imageHeight, downsampling);

            SavedBitmap = rgbImage;

            for (int x = 0; x < rgbImage.getWidth(); x++) {
                for (int y = 0; y < rgbImage.getHeight(); y++) { //why height / 2
                    int pixel = rgbImage.getPixel(x, y);
                    redValue += red(pixel);
                    blueValue += blue(pixel);
                }
            }

            if (redValue >= blueValue) {
                colorString = "RED";
            } else {
                colorString = "BLUE";
            }
        }

        return(colorString);
    }
}
