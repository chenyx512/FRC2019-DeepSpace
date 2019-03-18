package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.libs.*;

/**
 * This class is in charge of communicating with the rpi
 * The isConnected variable tells whether rpi is connected, and timestamp shows the last update of rpi
 * At each update, updateTarget method calculates the target coordinate, and also updates locked target if it is close to one of the calculated ones
 */
public class Vision{
    private static Vision instance;
    public static Vision getInstance() {
        if(instance==null)
            synchronized(Vision.class){
                if(instance==null)
                    instance=new Vision();
            }
        return instance;
    }
    private RobotState robotState=RobotState.getInstance();
    private Control control=Control.getInstance();
    private NetworkTableInstance ntinst;
    private NetworkTable outputTable;
    private NetworkTableEntry outputEntry;
    private Pose lockedTarget; //use getters and setters and NEVER use it without synch
    private volatile Pt primaryTarget;
    private volatile Pt[] potentialTargets;

    public volatile double timestamp=0, lastLockTime=0;
    public volatile boolean isConnected;
    

    private Vision() {
        ntinst = NetworkTableInstance.getDefault();
        outputTable = ntinst.getTable("piOutput");
        outputEntry = outputTable.getEntry("output");
        outputEntry.addListener(event->{
            updateTarget();
        }, EntryListenerFlags.kNew|EntryListenerFlags.kUpdate);
        Notifier notifier=new Notifier(()->{updateConnection();});
        notifier.startPeriodic(0.02);
    }

    private void updateConnection(){
        if(Timer.getFPGATimestamp()-timestamp>Constants.VISION_DISCONNECT_TIME){
            isConnected=false;
            setLockedTarget(null);
        } else
            isConnected=true;
        if(control.isLockTarget())
            setPrimaryLock();
    }

    private void updateTarget(){
        double[] output;
        output=outputEntry.getDoubleArray(new double[0]);
        if((output.length&3)!=1){
            System.out.printf("ERR output array length % 4 !=1\n");
            return;
        }
        timestamp=Timer.getFPGATimestamp()-output[0]-0.005;
        if(output.length==1){
            primaryTarget=null;
            potentialTargets=null;
            return;
        }
        int targetCnt=(output.length-1)/4;
        Pose pose=robotState.getPose(), curLockedTarget=getLockedTarget();
        Pt[] targets=new Pt[targetCnt];
        int lockIndex=-1;
        for(int i=0;i<targetCnt;i++){
            targets[i]=getCoordinate(output[i*4+1], output[i*4+3], output[i*4+4], pose);
            if(curLockedTarget!=null && lockIndex==-1 && output[i*4+2]<30 && targets[i].distance(curLockedTarget)<Constants.LOCK_ERROR)
            {
                lockIndex=i;
                // System.out.printf("update lock at time=%.4f\n",Timer.getFPGATimestamp());
            }
        }
        Pt[] newPotentialTargets=null;
        if(targetCnt>1){
            newPotentialTargets=new Pt[targetCnt-1];
            for(int i=1;i<targetCnt;i++)
                    newPotentialTargets[i-1]=targets[i];
        }
        primaryTarget=targets[0];
        // System.out.printf("x:%.2f y:%.2f\n",primaryTarget.x,primaryTarget.y);
        potentialTargets=newPotentialTargets;

        if(lockIndex!=-1){
            double targetL2RAngle = output[lockIndex*4+2];
            double targetLDis=Constants.VISION_DISTANCE_CONSTANT/output[4*lockIndex+3];
            double targetRDis=Constants.VISION_DISTANCE_CONSTANT/output[4*lockIndex+4];
            double L2RDis=Math.sqrt(targetLDis*targetLDis+targetRDis*targetRDis
                    -2*targetLDis*targetRDis*Math.cos(Math.toRadians(targetL2RAngle))); //law of cosine
            double Robot2R2LAngle=Math.toDegrees(Math.asin(targetLDis*Math.sin(Math.toRadians(targetL2RAngle))/L2RDis));
            if(targetLDis>targetRDis)
                Robot2R2LAngle=180-Robot2R2LAngle; // due to range of arcsin
            double properAngle=pose.theta+90-targetL2RAngle/2-Robot2R2LAngle;
            setLockedTarget(new Pose(targets[lockIndex], properAngle));
        }
    }

    private Pt getCoordinate(double x, double lh, double rh, Pose pose){
        double angle = Math.toRadians(pose.theta - x);
        double dis=(Constants.VISION_DISTANCE_CONSTANT/lh+Constants.VISION_DISTANCE_CONSTANT/rh)/2;
        return new Pt(pose.x+Math.cos(angle)*dis, pose.y+Math.sin(angle)*dis);
    }

    private final Object lockedTargetLock=new Object();
    public void setLockedTarget(Pose _lockedTarget){
        synchronized(lockedTargetLock){
            lockedTarget=_lockedTarget;
        }
    }
    public Pose getLockedTarget(){
        synchronized(lockedTargetLock){
            return lockedTarget;
        }
    }
    public Pt getPrimaryTarget(){
        return primaryTarget;
    }
    public Pt[] getPotentialTargets(){
        return potentialTargets;
    }
    public void setPrimaryLock(){
        Pt _primaryTarget=primaryTarget;
        if(_primaryTarget==null)
            setLockedTarget(null);
        else
            setLockedTarget(new Pose(_primaryTarget, Double.NaN));
    }
}