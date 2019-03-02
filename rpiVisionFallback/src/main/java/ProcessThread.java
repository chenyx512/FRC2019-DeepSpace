import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ProcessThread extends Thread{
    public static NetworkTableInstance ntinst;
    public static NetworkTableEntry outputEntry;
    public static CvSource outputStream;
    private static final Object entryLock=new Object(), streamLock=new Object();
    private static Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_CROSS, new Size(3,3));
    
    private Mat source;
    private double timestamp;
    private int threadNumber;
    private ArrayList<MatOfPoint> contours=new ArrayList<MatOfPoint>();
    private ArrayList<Pretarget>leftTargets=new ArrayList<Pretarget>(), rightTargets=new ArrayList<Pretarget>();
    private ArrayList<Target>targets=new ArrayList<Target>();


    public ProcessThread(int _threadNumber, Mat _source, double _timestamp){
        threadNumber=_threadNumber;
        source=_source;
        timestamp=_timestamp;
    }

    @Override
    public void run(){
        System.out.printf("thread %d got source time %f\n", threadNumber, timestamp);
        Imgproc.cvtColor(source, source, Imgproc.COLOR_BGR2HSV);
        Core.inRange(source, Constants.lowHSV, Constants.highHSV, source);
        Imgproc.findContours(source, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if(Constants.DEBUG)
            Imgproc.cvtColor(source, source, Imgproc.COLOR_GRAY2BGR);
        for(MatOfPoint contour: contours){
            if(Imgproc.contourArea(contour)<Constants.MIN_DETECT_AREA)
                continue;
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            if(Math.max(rect.size.width, rect.size.height)/Math.min(rect.size.width, rect.size.height)>Constants.MAX_W2H_RATIO)
                continue;
            double rectAngle = rect.size.width > rect.size.height ? rect.angle + 90 : rect.angle;
            if (Math.abs(rectAngle) < Constants.MIN_DETECT_ANGLE || Math.abs(rectAngle) > Constants.MAX_DETECT_ANGLE)
                continue;
            
            if (rectAngle<0)
                rightTargets.add(new Pretarget(Imgproc.boundingRect(contour), rect));
            else
                leftTargets.add(new Pretarget(Imgproc.boundingRect(contour), rect));
            if(Constants.DEBUG)
                drawRotatedRect(source, rect, new Scalar(255, 0, 0), 2);
            // Collections.sort(left);
            // Collections.sort(right);
        }
        if(leftTargets.size()>5 || rightTargets.size()>5){
            System.out.printf("too many targets\n");
            return;
        }
        System.out.printf("l%d r%d\n",leftTargets.size(), rightTargets.size());
        for (Pretarget ltarget : leftTargets)
            for (Pretarget rtarget : rightTargets) {
                if (ltarget.compareTo(rtarget)!=-1)
                    continue;
                double ratioDx2Length = (rtarget.minRect.center.x - ltarget.minRect.center.x) / (Math.max(ltarget.minRect.size.height, ltarget.minRect.size.width));
                double ratioDy2Length = Math.abs((rtarget.minRect.center.y - ltarget.minRect.center.y) / Math.max(ltarget.minRect.size.height, ltarget.minRect.size.width));
                if (ratioDx2Length > Constants.MAX_DX2LENGTH || ratioDy2Length > Constants.MAX_DY2LENGTH)
                    continue;
                double lx=calculateAngle(ltarget.rect.x+ltarget.rect.width),rx=calculateAngle(rtarget.rect.x);
                if(Constants.DEBUG)
                    Imgproc.circle(source, new Point((ltarget.minRect.center.x+rtarget.minRect.center.x)/2,(ltarget.minRect.center.y+rtarget.minRect.center.y)/2), 3, new Scalar(0, 0, 255), -1);
                targets.add(new Target((lx+rx)/2, rx-lx, ltarget.rect.height/Constants.HEIGHT, rtarget.rect.height/Constants.HEIGHT));
                break;
            }
        Collections.sort(targets);
        double[] output=new double[targets.size()*4+1];
        for(int i=0;i<targets.size();i++){
            Target target=targets.get(i);
            output[i*4+1]=target.x;
            output[i*4+2]=target.dx;
            output[i*4+3]=target.lh;
            output[i*4+4]=target.rh;
        }
        putFrame(source);
        setEntry(output, timestamp);
    }

    private static void setEntry(double[] array, double acquireTime){
        synchronized(entryLock){
            array[0]=System.currentTimeMillis()/1000.0-acquireTime;
            outputEntry.setDoubleArray(array);
            ntinst.flush();
        }
    }

    private static void putFrame(Mat mat){
        synchronized(streamLock){
            outputStream.putFrame(mat);
        }
    }

    private double calculateAngle (double x){
        return Math.toDegrees(Math.atan((x-Constants.WIDTH/2)/Constants.FOCAL_LENGTH_HORIZONTAL));
    }


    private static void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color, int thickness) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);
        MatOfPoint points = new MatOfPoint(vertices);
        Imgproc.drawContours(image, Arrays.asList(points), -1, color, thickness);
    }

    private class Pretarget implements Comparable<Pretarget>{
        public Rect rect;
        public RotatedRect minRect;

        Pretarget(Rect _rect, RotatedRect _minRect){
            rect=_rect;
            minRect=_minRect;
        }

        public int compareTo(Pretarget other){
            if(minRect.center.x==other.minRect.center.x)
                return 0;
            return minRect.center.x<other.minRect.center.x? -1:0;
        }
    }
    private class Target implements Comparable<Target>{
        public double x, dx, lh, rh;
        public Target(double _x, double _dx, double _lh, double _rh){
            x=_x;
            dx=_dx;
            lh=_lh;
            rh=_rh;
        }
        public int compareTo(Target other){
            if(Math.abs(x)==Math.abs(other.x))
                return 0;
            return Math.abs(x)<Math.abs(other.x)? -1:1;
        }
    }
}