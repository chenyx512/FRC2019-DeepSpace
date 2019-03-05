package frc.robot;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.libs.*;

/**
 * This class is in charge of displaying critical info on driving camera
 */
public class HUD{
    private static HUD instance;
    public static HUD getInstance() {
        if(instance==null)
            synchronized(HUD.class){
                if(instance==null)
                    instance=new HUD();
            }
        return instance;
    }

    private Vision vision=Vision.getInstance();
    public volatile String[] messages={null, null, null};
    private static final Point[] messagePoints={new Point(0,20), new Point(0,60), new Point(0,100)};

    private HUD(){
        new Thread(() -> {
                UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
                camera.setResolution(Constants.HUD_WIDTH, Constants.HUD_HEIGHT);
                camera.setExposureManual(40);
                camera.setWhiteBalanceManual(5000);
                camera.setFPS(20);
                
                CvSink cvSink = CameraServer.getInstance().getVideo();
                CvSource outputStream = CameraServer.getInstance().putVideo("HUD", Constants.HUD_WIDTH, Constants.HUD_HEIGHT);

                Mat source = new Mat();

                while(!Thread.interrupted()) {
                    long err=cvSink.grabFrame(source);
                    if(err==0)
                    {
                        System.out.printf("timeout acq cam\n");
                        continue;
                    }
                    Pose lockedTarget=vision.getLockedTarget();
                    Pt primaryTarget=vision.getPrimaryTarget();
                    Pt[] potentialTargets=vision.getPotentialTargets();
                    Pose pose=RobotState.getInstance().getPose();

                    if(primaryTarget!=null)
                        Imgproc.circle(source, getHUDAngle(primaryTarget, pose), 4, new Scalar(0,128,255), -1);
                    if(potentialTargets!=null)
                        for(Pt target : potentialTargets)
                            Imgproc.circle(source, getHUDAngle(target, pose), 4, new Scalar(0,255,128), -1);
                    if(lockedTarget!=null){
                        Point center=getHUDAngle(lockedTarget, pose);
                        Point c1=new Point(center.x-4, center.y-4);
                        Point c2=new Point(center.x+4, center.y+4);
                        Imgproc.rectangle(source, c1, c2, new Scalar(0, 0, 255), 2);
                    }
                        
                    for(int i=0;i<messages.length;i++)
                        if(messages[i]!=null)
                            Imgproc.putText(
                                source,                          // Matrix obj of the image
                                messages[i],          // Text to be added
                                messagePoints[i],               // point
                                Core.FONT_HERSHEY_TRIPLEX,      // front face
                                1,                               // front scale
                                new Scalar(0, 0, 255),             //BGR
                                2,  //thickness
                                4,  //line type
                                false   //top left origin
                            );
                    outputStream.putFrame(source);
                }
            }).start();
    }

    private Point getHUDAngle(Pt pt, Pose robotPose){
        double x=robotPose.theta-Math.toDegrees(Math.atan2(pt.y-robotPose.y, pt.x-robotPose.x));
        x=((x%360)+360)%360;
        if(x>180)x-=360;
        // System.out.printf("x %.1f\n",x);
        // x=Constants.HUD_WIDTH/2+Constants.HUD_WIDTH/2/Math.sin(Math.toRadians(Constants.HUD_HORIZONTAL_FOV/2))*Math.sin(Math.toRadians(toRight));
        x=Constants.HUD_WIDTH*(x/Constants.HUD_HORIZONTAL_FOV+0.5);
        x=Math.max(0,Math.min(x, Constants.HUD_WIDTH));
        double y=Constants.HUD_HEIGHT*(1-pt.distance(robotPose)/Constants.HUD_MAX_SHOWN_DISTANCE);
        return new Point(x,y);
    }
}