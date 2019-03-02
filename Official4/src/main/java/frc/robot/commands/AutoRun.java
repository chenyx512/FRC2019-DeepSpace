package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.libs.*;

public class AutoRun extends Command {
    public double P = 5e-3, I = 3e-4, D = 3.2e-5;
    private double integral, previousError, derivative;
    private double toTargetTheta, angularPower = 0, angularError = 0;
    private boolean isShoot;
    private RobotState robotState = RobotState.getInstance();
    private Vision vision = Vision.getInstance();
    private Control control=Control.getInstance();
    private Pose pose, target;
    private double toTargetDis;
    private HUD hud=HUD.getInstance();

    public AutoRun() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        this.setTimeout(20);
        vision.setPrimaryLock();
        if(!isContinue())
            return;
        previousError=angularError;
        isShoot = false;
        previousError=Double.NaN;
        Robot.hatchArm.isSlideForward = false;
        Robot.hatchArm.isHatchGrabberOpen = true;
    }
    
    @Override
    protected void execute() {
        if(!isContinue())
            return;
        SmartDashboard.putNumber("error", angularError);
        toTargetDis=pose.distance(target);
        System.out.printf("toTarget:%.1f\n",toTargetDis);
        if ( toTargetDis<Constants.SHOOT_DIS && angularError<Constants.MAX_ALLOWED_ANGLE_ERROR 
                ||  toTargetDis<Constants.SHOOT_DIS-100 && !isShoot
                || control.getAutoShoot()&&!isShoot) {
            System.out.println("!!!!!!!!!!!!! shoot !!!!!!!!!!!!!");
            isShoot = true;
            Robot.hatchArm.isSlideForward = true;
            Robot.hatchArm.isHatchGrabberOpen = false;
            this.setTimeout(this.timeSinceInitialized() + 0.7);
        }
        calculateAngularPower();
        double forwardSpeed = Math.max(0,calculateForwardPower());
        double angleOverride=control.getAutoAngleOverride()/4.0;
        Robot.driveTrain.drive.arcadeDrive(forwardSpeed*control.getAutoSpeedCo(), angularPower+angleOverride, false);
        
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
        return true;
    }

    void calculateAngularPower() {
        integral += (angularError * 0.02);
        derivative = Double.isNaN(previousError)? 0:(angularError - previousError) / 0.02;
        previousError=angularError;
        if (Math.abs(angularPower) > 1 || integral * angularError < 0 || Math.abs(angularError)>5)
            integral = 0;
        angularPower = P * angularError + I * integral + D * derivative;
        double linSpeed=Robot.driveTrain.getLinearSpeed(), min_turn;
        if(Math.abs(linSpeed)<10)
            min_turn=Constants.MIN_TURN_SPEED*2.5;
        else{
            min_turn=Constants.MIN_TURN_SPEED;
            // min_turn=Math.abs(linSpeed/300);
            // min_turn=10*Math.pow(0.5*min_turn-0.57, 4);
            // min_turn=Math.min(1, Math.max(0.3, min_turn))*Constants.MIN_TURN_SPEED;
        }
        SmartDashboard.putNumber("minTurn", min_turn);
        // SmartDashboard.putNumber("lin_speed", Robot.driveTrain.getLinearSpeed());
        if (Math.abs(angularError) > Constants.MAX_ALLOWED_ANGLE_ERROR && Math.abs(angularPower) < min_turn)
        {
            angularPower = Math.signum(angularError)*min_turn;
            integral=0;
        }
    }

    private double calculateForwardPower(){
        hud.messages[1]=String.format("%.1f",(toTargetDis-Constants.SHOOT_DIS)/1000);
        if(toTargetDis<Constants.CLOSE_DIS || control.isAutoClose()){
            hud.messages[0]=isShoot? "ST":"IC";
            return Constants.CLOSE_SPEED-Math.abs(angularError)*Constants.CLOSE_ANGULAR_ERROR_PENALTY;
        }
        hud.messages[0]=isShoot? "ST":"A";
        double k=(toTargetDis-Constants.CLOSE_DIS)/Constants.CLOSE_DIS;
        if(k>1)
            k=1;
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