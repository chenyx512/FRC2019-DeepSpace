package frc.robot.chenyxVision;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.chenyxVision.libs.*;

public class AutoRun extends Command {
    private RobotState robotState = RobotState.getInstance();
    private Vision vision = Vision.getInstance();

    private double P = 0.025;
    private double toTargetDis, toTargetTheta, angularError = 0;
    private boolean isShoot;
    private Pose pose, target;

    public AutoRun() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void initialize() {
        vision.setPrimaryLock();
        if (!isContinue())
            return;
    }

    @Override
    protected void execute() {
        if (!isContinue())
            return;
        toTargetDis = pose.distance(target);
        if (toTargetDis < Constants.SHOOT_DIS) {
            System.out.printf("!!! SHOOT !!! delta: %.1f time:%.1f\n", angularError, this.timeSinceInitialized());
            isShoot = true;
            this.setTimeout(this.timeSinceInitialized() + 0.3);
        }
        Robot.driveTrain.drive.arcadeDrive(calculateForwardPower(), calculateAngularPower(), false);
    }

    private boolean isContinue() {
        if (this.isCanceled())
            return false;
        target = vision.getLockedTarget();
        if (target == null) {
            System.out.printf("lock target is null, auto returns\n");
            this.cancel();
            return false;
        }
        pose = robotState.getPose();
        if (pose == null) {
            System.out.printf("somehow pose==null, auto returns\n");
            this.cancel();
            return false;
        }
        toTargetTheta = Math.toDegrees(Math.atan2(target.y - pose.y, target.x - pose.x)) + compensateOffset();
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
    private double calculateForwardPower() {
        if (toTargetDis < Constants.CLOSE_DIS)
            if (Math.abs(angularError) > Constants.MAX_ALLOWED_ANGLE_ERROR * 2.5 && !isShoot)
                return 0;
            else
                return Constants.CLOSE_SPEED - Math.abs(angularError) * Constants.CLOSE_ANGULAR_ERROR_PENALTY;
        double k = Math.min(1, (toTargetDis - Constants.CLOSE_DIS) / Constants.CLOSE_DIS);
        if (Math.abs(angularError) > 6 * (1 + k) && !isShoot)
            return 0;
        else
            return Constants.CLOSE_SPEED + Constants.START_ADDITION_SPEED * k
                    - Math.abs(angularError) * Constants.CLOSE_ANGULAR_ERROR_PENALTY * k;
    }

    double calculateAngularPower() {
        double angularPower = P * angularError, linSpeed=getLinearSpeed(); //TODO implement this
        // if the robot is moving, we need greater power to get it going
        double min_turn = Constants.MIN_TURN_SPEED * (Math.abs(linSpeed) < 10 ? 2.5 : 1);
        if (Math.abs(angularError) > Constants.MAX_ALLOWED_ANGLE_ERROR && Math.abs(angularPower) < min_turn)
            angularPower = Math.signum(angularError) * min_turn;
        return angularPower;
    }

    /**
     * This method compensates for: 
     * 1) The physical offset of the camera relative to the arm 
     * 2) The needed compensation due to the robot going in at an angle not normal to the hatch, 
     * eg: when the robot is too much to the left, to needs to turn right a bit for the hatch not to hit the left bolts
     */
    private double compensateOffset() {
        double k = Math.max(0, Math.min(1, 1 - (toTargetDis - Constants.CLOSE_DIS) / Constants.CLOSE_DIS / 2));
        if (Double.isNaN(target.theta))
            return -Constants.PHYSICAL_OFFSET * k;
        double RV = ((target.theta - toTargetTheta) % 360 + 360) % 360;
        if (RV > 180)
            RV -= 360;
        RV *= -1;
        if (Math.abs(RV) > Constants.OFFSET_THRESH)
            return Math.signum(RV) * Constants.OFFSET_COMP - Constants.PHYSICAL_OFFSET * k;
        else
            return -Constants.PHYSICAL_OFFSET * k;
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