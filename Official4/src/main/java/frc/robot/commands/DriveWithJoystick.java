package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

/**
 * If the quickTurn button is pressed, this uses the talon to control the rotation SPEED so the robot rotates IN PLACE.
 * Therefore, pressing quickTurn button while driving fast with the ramp low may fall the robot.
 */
public class DriveWithJoystick extends Command {
    private static Control control=Control.getInstance();
    
    public DriveWithJoystick() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void execute() {
        double linearSpeed = 0.65 *control.getForwardThrottle();
        double rotationSpeed = (control.isQuickTurn()? 0.38:0.68)*control.getRotationThrottle();
        double backingComp=control.getForwardThrottle()<0 && !control.isQuickTurn()? -1:1;
        if(!control.isQuickTurn())
            Robot.driveTrain.drive.curvatureDrive(linearSpeed, backingComp * rotationSpeed, false);
        else{
            Robot.driveTrain.leftMaster.set(ControlMode.Velocity, rotationSpeed*300);
            Robot.driveTrain.rightMaster.set(ControlMode.Velocity, -rotationSpeed*300);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}