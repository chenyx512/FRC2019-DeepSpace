package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.Vision;
import frc.robot.libs.*;
import frc.robot.trajectory.*;

public class FollowTrajectory extends Command {
    private QuinticSpline spline;
    private Trajectory trajectory;
    private Vision vision=Vision.getInstance();
    
    private Object lock=new Object();
    private boolean isCalculated=false;
    private double startTime=0;
    private RobotState robotState=RobotState.getInstance();

    public FollowTrajectory() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        Robot.driveTrain.drive.setSafetyEnabled(false);
        if(!vision.isConnected || !vision.isFindTarget){
            System.out.printf("no target/connection\n");
            this.cancel();
            return;
        }
        Pose currentPose = robotState.getCurrentPose();
        double angle=vision.targetProperAngle;
        //double angle=90;
        double disInFront=900;
        Pose targetPose = new Pose(vision.target.x-disInFront*Math.cos(Math.toRadians(angle)), 
           vision.target.y-disInFront*Math.sin(Math.toRadians(angle)), angle);
        // Pose targetPose = new Pose(currentPose.x+Robot.trajectoryY*Math.cos(Math.toRadians(currentPose.theta))+Robot.trajectoryX*Math.cos(Math.toRadians(currentPose.theta-90)),
        //                         currentPose.y+Robot.trajectoryY*Math.sin(Math.toRadians(currentPose.theta))+Robot.trajectoryX*Math.sin(Math.toRadians(currentPose.theta-90)),
        //                         currentPose.theta+Robot.trajectoryTheta);
        isCalculated=false;
        System.out.printf("start generating thread\n");
        Thread calculation = new Thread(() -> {
            spline=new QuinticSpline(currentPose, targetPose);
            trajectory=new Trajectory(spline, robotState.getCurrentLinearSpeed(), 0);
            System.out.printf("end generating traj\n");
            synchronized(lock){
                isCalculated=true;
                startTime=this.timeSinceInitialized();
            }
        });
        calculation.start();
    }

    @Override
    protected void execute() {
        if(this.isCanceled())
            return;
        synchronized(lock){
            if(!isCalculated)return;
        }
        MotionPose profile = trajectory.profileMap.getInterpolated(new InterpolatingDouble(this.timeSinceInitialized()-startTime));
        Pose pose=robotState.getCurrentPose();
        Vector speed=Kinematics.getWheelDynamics(profile.v, profile.getOmiga());
        Vector acc= Kinematics.getWheelDynamics(profile.a, profile.a*profile.curvature+profile.v*profile.v*profile.dcurvatureds);
        if(Robot.loopCnt%5==0)
        {
            System.out.printf("t:%.2f aT:%.2f Vl:%.1f Vr:%.1f Al:%.1f Ar:%.1f\n",this.timeSinceInitialized()-startTime,profile.time,speed.x,speed.y,acc.x,acc.y);
            System.out.printf("Ex:%.1f Ey=%.1f Etheta=%.1f\nAx=%.1f Ay=%.1f Atheta=%.1f\n",profile.x,profile.y,profile.theta,pose.x,pose.y,pose.theta);
        }
        Robot.driveTrain.leftMaster.set(ControlMode.Velocity, speed.x/10, DemandType.ArbitraryFeedForward, acc.x*Constants.Aff);
        Robot.driveTrain.rightMaster.set(ControlMode.Velocity, speed.y/10, DemandType.ArbitraryFeedForward, acc.y*Constants.Aff);
    }

    @Override
    protected boolean isFinished() {
        synchronized(lock){
            if(!isCalculated)return false;
            return this.timeSinceInitialized()-startTime>trajectory.time;
        }
    }

    @Override
    protected void end() {
        Robot.driveTrain.drive.setSafetyEnabled(true);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
