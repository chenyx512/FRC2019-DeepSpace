import org.opencv.core.Scalar;

public class Constants{
    // if DEBUG==true, the output stream will be converted to 3-channel image and marked with recognized targets, otherwize output stream is just HSV threshholded
    public static boolean DEBUG=false;
    
    public static double FOCAL_LENGTH_HORIZONTAL;
    public static double FOCAL_LENGTH_VERTICAL;
    public static double WIDTH;
    public static double HEIGHT;
    public static double startTime;

    // these two are physically measured
    public static final Scalar lowHSV=new Scalar(43, 24, 24);
    public static final Scalar highHSV=new Scalar(77, 255, 160);
    
    // following are threshholds for target recognition, needs to be tuned since it doesn't work very well in our competition
    public static final double MIN_DETECT_AREA = 20;
    public static final double MIN_DETECT_ANGLE = 5;
    public static final double MAX_DETECT_ANGLE = 32;
    public static final double MAX_DX2LENGTH=4;
    public static final double MAX_DY2LENGTH=0.75;
    public static final double MAX_W2H_RATIO = 5;

    // these two are physically measured data
    public static final double VERTICAL_FOV_DEG = 34.98;
    public static final double HORIZONTAL_FOV_DEG = 59.3;

    public static final int PROCESS_THREAD_NUMBER = 2; // 1-3 seem to be reasonable, as like rpi 3b+ has 4 cores
}