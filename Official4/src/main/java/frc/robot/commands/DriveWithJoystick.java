package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

public class DriveWithJoystick extends Command {
    private static Control control=Control.getInstance();
    
    public DriveWithJoystick() {
        requires(Robot.driveTrain);
    }

    @Override
    protected void execute() {
        double linearSpeed = control.getSpeed();
        double rotationSpeed = control.getSpeed() * (control.isQuickTurn()? 0.7:1);
        double backingComp=control.getForwardThrottle()<0 && !control.isQuickTurn()? -1:1;
        Robot.driveTrain.drive.curvatureDrive(control.getForwardThrottle() * linearSpeed,
                control.getRotationThrottle() * backingComp * rotationSpeed, control.isQuickTurn());
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