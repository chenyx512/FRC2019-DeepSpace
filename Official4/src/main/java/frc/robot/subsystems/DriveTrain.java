package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.DriveWithJoystick;

public class DriveTrain extends Subsystem {
    private static WPI_TalonSRX leftMaster, rightMaster, leftSlave, rightSlave;
    public static DifferentialDrive drive;

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
    }

    @Override
    public void periodic(){
        if((Robot.loopCnt&7)==0){
            SmartDashboard.putNumber("left", leftMaster.getSelectedSensorPosition());
            SmartDashboard.putNumber("right", rightMaster.getSelectedSensorPosition());
            SmartDashboard.putNumber("leftV", leftMaster.getSelectedSensorVelocity());
            SmartDashboard.putNumber("rightV", rightMaster.getSelectedSensorVelocity());
        }
    }

    public DriveTrain() {
        leftMaster = new WPI_TalonSRX(3);
        configTalon(leftMaster);
        rightMaster = new WPI_TalonSRX(4);
        configTalon(rightMaster);
        leftSlave = new WPI_TalonSRX(1);
        configTalon(leftSlave);
        rightSlave = new WPI_TalonSRX(2);
        configTalon(rightSlave);

        configMaster(leftMaster);
        configMaster(rightMaster);
        rightMaster.setSensorPhase(true);
        leftMaster.setSensorPhase(true);
        rightSlave.setInverted(true);
        rightMaster.setInverted(true);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
        leftMaster.configSelectedFeedbackCoefficient(0.325);
        rightMaster.configSelectedFeedbackCoefficient(0.325);

        drive = new DifferentialDrive(leftMaster, rightMaster);
        drive.setRightSideInverted(false);
        drive.setExpiration(0.4);
    }

    private void configTalon(WPI_TalonSRX talon) {
        talon.configFactoryDefault();
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configOpenloopRamp(Constants.DRIVETRAIN_DEFAULT_OPEN_RAMP);
        talon.configVoltageCompSaturation(Constants.TALON_MAX_VOLTAGE);
        talon.enableVoltageCompensation(true);
        // current limit
        talon.configContinuousCurrentLimit(Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT);
        talon.configPeakCurrentDuration(Constants.DRIVE_PEAK_CURRENT_DURANTION);
        talon.configPeakCurrentLimit(Constants.DRIVE_PEAK_CURRENT_LIMIT);
        talon.enableCurrentLimit(true);
        // max output
        talon.configNominalOutputForward(0);
        talon.configPeakOutputForward(1);
        talon.configPeakOutputReverse(-1);
    }
    
    private void configMaster(WPI_TalonSRX master) {
        master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        master.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
        master.configVelocityMeasurementWindow(64);
        master.config_kP(Constants.DRIVETRAIN_VELOCITY_SLOT, Constants.DRIVETRAIN_VELOCITY_GAINS.kP);
		master.config_kI(Constants.DRIVETRAIN_VELOCITY_SLOT, Constants.DRIVETRAIN_VELOCITY_GAINS.kI);
		master.config_kD(Constants.DRIVETRAIN_VELOCITY_SLOT, Constants.DRIVETRAIN_VELOCITY_GAINS.kD);
		master.config_kF(Constants.DRIVETRAIN_VELOCITY_SLOT, Constants.DRIVETRAIN_VELOCITY_GAINS.kF);
		master.config_IntegralZone(Constants.DRIVETRAIN_VELOCITY_SLOT, Constants.DRIVETRAIN_VELOCITY_GAINS.kIzone);
		master.configClosedLoopPeakOutput(Constants.DRIVETRAIN_VELOCITY_SLOT, Constants.DRIVETRAIN_VELOCITY_GAINS.kPeakOutput);
        master.selectProfileSlot(Constants.DRIVETRAIN_VELOCITY_SLOT, 0);
        master.configAllowableClosedloopError(Constants.DRIVETRAIN_VELOCITY_SLOT, 0);

        master.configClosedloopRamp(Constants.DEFAULT_CLOSED_RAMP);
    }

    public void openRamp(double ramp){
        leftMaster.configOpenloopRamp(ramp);
        rightMaster.configOpenloopRamp(ramp);
        leftSlave.configOpenloopRamp(ramp);
        rightSlave.configOpenloopRamp(ramp);
    }

    public synchronized int getLeftPosition(){
        return leftMaster.getSelectedSensorPosition();
    }
    public synchronized int getRightPosition(){
        return rightMaster.getSelectedSensorPosition();
    }
    public synchronized int getLinearSpeed(){
        return leftMaster.getSelectedSensorVelocity()*5+rightMaster.getSelectedSensorVelocity()*5;
    }
    public WPI_TalonSRX getPigeonTalon(){
        return rightSlave;
    }
}
