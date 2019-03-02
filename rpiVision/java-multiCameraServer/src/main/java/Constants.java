import org.opencv.core.Scalar;

public class Constants{
    public static boolean DEBUG=false;
    public static double FOCAL_LENGTH_HORIZONTAL;
    public static double FOCAL_LENGTH_VERTICAL;
    public static double WIDTH;
    public static double HEIGHT;
    
    public static double startTime;

    public static final Scalar lowHSV=new Scalar(43, 24, 24);
    public static final Scalar highHSV=new Scalar(77, 255, 160);

    public static final double MIN_DETECT_AREA = 20;
    public static final double MIN_DETECT_ANGLE = 5;
    public static final double MAX_DETECT_ANGLE = 32;
    public static final double MAX_DX2LENGTH=4;
    public static final double MAX_DY2LENGTH=0.75;
    public static final double MAX_W2H_RATIO = 5;

    public static final double VERTICAL_FOV_DEG = 34.98;
    public static final double HORIZONTAL_FOV_DEG = 59.3;

    public static final int PROCESS_THREAD_NUMBER = 1;
}