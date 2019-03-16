package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.libs.*;

/**
 * Before calling Command.start, set the isGet variable first.
 * it is actually better to change from pure P control to a arc to the target
 * I and D terms are not used here but are used by talon velocity control
 */
public class AutoRun extends Command {
    public boolean isGet=false;

    private RobotState robotState = RobotState.getInstance();
    private Vision vision = Vision.getInstance();
    private Control control=Control.getInstance();
    private HUD hud=HUD.getInstance();

    private double P=8.5, I=0.3;
    private double toTargetDis, toTargetTheta, angularPower = 0, angularError = 0;
    private boolean isShoot;
    private Pose pose, target;
    private double linSpeed;

    public AutoRun() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        this.setTimeout(20);
        vision.setPrimaryLock();
        if(!isContinue())
            return;
        isShoot = false;
        Robot.hatchArm.isSlideForward = isGet;
        Robot.hatchArm.isHatchGrabberOpen = !isGet;
    }
    
    @Override
    protected void execute() {
        if(!isContinue())
            return;
        toTargetDis=pose.distance(target);
        linSpeed=Robot.driveTrain.getLinearSpeed();
        if (isShoot()) {
            System.out.println("!!!!!!!!!!!!! shoot !!!!!!!!!!!!!");
            isShoot = true;
            if(!isGet)
                Robot.hatchArm.isSlideForward = true;
            Robot.hatchArm.isHatchGrabberOpen = isGet;
            this.setTimeout(this.timeSinceInitialized() + (isGet? 0.15:0.45) );
        }
        angularPower=P*angularError;
        double forwardSpeed = Math.max(0,calculateForwardPower())*control.getAutoSpeedCo() * 2400;
        Robot.driveTrain.leftMaster.set(ControlMode.Velocity, (forwardSpeed+angularPower)/10);
        Robot.driveTrain.rightMaster.set(ControlMode.Velocity, (forwardSpeed-angularPower)/10);
        hud.messages[0] = (isGet? "G":"P")+(isShoot? "S":"");
    }

    private boolean isShoot(){
        double predictedDis=toTargetDis-Constants.SHOOT_DIS-(isGet? 0:linSpeed*0.4);
        hud.messages[1]=String.format("%.1f",predictedDis/1000);
        return predictedDis<Constants.SHOOT_DIS && !isShoot;
    }

    private boolean isContinue(){
        if(this.isCanceled())
            return false;
        target=vision.getLockedTarget();
        if(target==null){
            System.out.printf("lock target is null, auto returns\n");
            this.cancel();
            return false;
        }
        pose=robotState.getPose();
        if(pose==null){
            System.out.printf("somehow pose==null, auto returns\n");
            this.cancel();
            return false;
        }
        toTargetTheta = Math.toDegrees(Math.atan2(target.y - pose.y, target.x - pose.x))+compensateOffset();
        angularError = robotState.getPose().theta - toTargetTheta;
        angularError = ((angularError % 360) + 360) % 360;
        if (angularError > 180)
            angularError -= 360;
        SmartDashboard.putNumber("error", angularError);
        return true;
    }

    /**
     * The closer and the more disaligned the robot is, the slower it should go
     */
    private double calculateForwardPower(){
        if(toTargetDis<Constants.CLOSE_DIS)
            if(Math.abs(angularError)>6 && !isShoot)return 0;
            else return Constants.CLOSE_SPEED-Math.abs(angularError)*Constants.CLOSE_ANGULAR_ERROR_PENALTY;
        double k=Math.min(1 , (toTargetDis-Constants.CLOSE_DIS) / Constants.CLOSE_DIS);
        if(Math.abs(angularError)>6*(1+k) && !isShoot)
            return 0;
        else return Constants.CLOSE_SPEED+Constants.START_ADDITION_SPEED*k-Math.abs(angularError)*Constants.CLOSE_ANGULAR_ERROR_PENALTY*k;
    }

    /**
     * This method compensates for:
     * 1) The physical offset of the camera relative to the arm
     * 2) The needed compensation due to the robot going in at an angle not normal to the hatch,
     *         eg: when the robot is too much to the left, to needs to turn right a bit for the hatch not to hit the left bolts
     */
    private double compensateOffset(){
        double k=Math.max(0,Math.min(1 , 1-(toTargetDis-Constants.CLOSE_DIS) / Constants.CLOSE_DIS / 2));;
        if(Double.isNaN(target.theta) || isGet)
            return -Constants.PHYSICAL_OFFSET*k;
        double RV= ((target.theta-toTargetTheta)%360+360)%360;
        if(RV>180)RV-=360;
        RV*=-1;
        if(Math.abs(RV)>Constants.OFFSET_THRESH && isGet==false)
            return Math.signum(RV)*Constants.OFFSET_COMP-Constants.PHYSICAL_OFFSET*k;
        else return -Constants.PHYSICAL_OFFSET*k;
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
    @Override
    protected void end() {
        Robot.hatchArm.isSlideForward=false;
        System.out.printf("auto finish with delta: %.1f time:%.1f\n", angularError, this.timeSinceInitialized());
    }
    @Override
    protected void interrupted() {
        System.out.println("auto interupted");
        end();
    }
}