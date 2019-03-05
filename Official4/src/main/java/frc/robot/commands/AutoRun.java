package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.libs.*;

public class AutoRun extends Command {
    public boolean isGet=false;

    private RobotState robotState = RobotState.getInstance();
    private Vision vision = Vision.getInstance();
    private Control control=Control.getInstance();
    private HUD hud=HUD.getInstance();

    private double P=0.02;
    private double toTargetDis, toTargetTheta, angularPower = 0, angularError = 0;
    private boolean isShoot;
    private Pose pose, target;
    double linSpeed;

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
            Robot.hatchArm.isSlideForward = !isGet;
            Robot.hatchArm.isHatchGrabberOpen = isGet;
            this.setTimeout(this.timeSinceInitialized() + 0.7);
        }
        double forwardSpeed = Math.max(0,calculateForwardPower());
        double angleOverride=control.getAutoAngleOverride()/4.0;
        Robot.driveTrain.drive.arcadeDrive(forwardSpeed*control.getAutoSpeedCo(), angularPower+angleOverride, false);
        hud.messages[0] = (isGet? "G":"P")+(isShoot? "S":"");
    }

    private boolean isShoot(){
        double predictedDis=toTargetDis-Constants.SHOOT_DIS-linSpeed*0.5;
        hud.messages[1]=String.format("%.1f",predictedDis/1000);
        return predictedDis<Constants.SHOOT_DIS;
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
        toTargetTheta = Math.toDegrees(Math.atan2(target.y - pose.y, target.x - pose.x));
        angularError = robotState.getPose().theta - toTargetTheta;
        angularError = ((angularError % 360) + 360) % 360;
        if (angularError > 180)
            angularError -= 360;
        SmartDashboard.putNumber("error", angularError);
        return true;
    }

    void calculateAngularPower() {
        angularPower=P*angularError;
        double min_turn;
        if(Math.abs(linSpeed)<10)
            min_turn=Constants.MIN_TURN_SPEED*2.5;
        else{
            min_turn=Constants.MIN_TURN_SPEED;
            // min_turn=Math.abs(linSpeed/300);
            // min_turn=10*Math.pow(0.5*min_turn-0.57, 4);
            // min_turn=Math.min(1, Math.max(0.3, min_turn))*Constants.MIN_TURN_SPEED;
        }
        SmartDashboard.putNumber("minTurn", min_turn);
        if (Math.abs(angularError) > Constants.MAX_ALLOWED_ANGLE_ERROR && Math.abs(angularPower) < min_turn)
            angularPower = Math.signum(angularError)*min_turn;
    }

    private double calculateForwardPower(){
        if(Math.abs(angularError)>7 && !isShoot)return 0;
        if(toTargetDis<Constants.CLOSE_DIS)
            return Constants.CLOSE_SPEED-Math.abs(angularError)*Constants.CLOSE_ANGULAR_ERROR_PENALTY;
        double k=Math.min(1 , (toTargetDis-Constants.CLOSE_DIS) / Constants.CLOSE_DIS);
        return Constants.CLOSE_SPEED+Constants.START_ADDITION_SPEED*k-Math.abs(angularError)*Constants.CLOSE_ANGULAR_ERROR_PENALTY*k;
    }

    @Override
    protected boolean isFinished() {
        return this.isTimedOut();
    }
    @Override
    protected void end() {
        System.out.printf("auto finish with delta: %.1f time:%.1f\n", angularError, this.timeSinceInitialized());
    }
    @Override
    protected void interrupted() {
        System.out.println("auto interupted");
        end();
    }
}

// private double compensateOffset(double currentTheta){
    //     if(Double.isNaN(target.theta))
    //         return 0;
    //     double RV= ((target.theta-toTargetTheta)%360+360)%360;
    //     if(RV>180)RV-=360;
    //     if(toTargetDis<Constants.CLOSE_DIS){
    //         RV/=-10;
    //         if(Math.abs(RV)>4)
    //             RV=4*Math.signum(RV);
    //         System.out.printf("RV:%.1f\n", RV);
    //     }
    //     else{
    //         if(RV<5)return 0;
    //         RV*=-0.8;
    //         if(Math.abs(RV)>25)
    //             RV=25*Math.signum(RV);
    //         if(toTargetDis<2*Constants.CLOSE_DIS)
    //             RV*=toTargetDis/Constants.CLOSE_DIS-1;
    //     }
    //     SmartDashboard.putNumber("offset", RV);
    //     return RV+Constants.AUTO_ANGULAR_COMPENSATION;
    // }

    
    // double overrideAngle = SmartDashboard.getNumber("overrideTheta", 0);
    // // above is for testing angle turning, remember to check forwardSpeed as well when use
    // if (Math.abs(overrideAngle) > 0.1) {
    //     targetTheta = robotState.getCurrentPose().theta - overrideAngle;
    //     angularError = previousError = getDelta();
    //     System.out.printf("override angle\n");
    //     return;
    // } else if (!vision.isConnected) {
    //     System.out.println("pi disconnected, command returns");
    //     this.cancel();
    //     return;
    // } else if (!vision.isFindTarget) {
    //     System.out.println("no unique target found, command returns");
    //     this.cancel();
    //     return;
    // }