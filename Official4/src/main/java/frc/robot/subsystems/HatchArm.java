package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Control;
import frc.robot.HUD;

public class HatchArm extends Subsystem {
    private WPI_TalonSRX armMaster, armSlave;
    private Solenoid hatchGrabber, slide;
    private Control control=Control.getInstance();
    private HUD hud=HUD.getInstance();
    public boolean isHatchGrabberOpen=true, isSlideForward=false;
    private boolean isOverrideArm=false;
    

    @Override
    public void initDefaultCommand() {
    }
    public HatchArm(){
        armMaster=new WPI_TalonSRX(7);
        configTalon(armMaster);
        configMaster(armMaster);

        armSlave=new WPI_TalonSRX(8);
        configTalon(armSlave);
        armSlave.setInverted(true);
        armSlave.follow(armMaster);

        hatchGrabber=new Solenoid(0,1);
        slide=new Solenoid(0,0);
        armMaster.set(ControlMode.PercentOutput, 0);
    }
    
    @Override
    public void periodic(){
        if(control.isChangeHatchGrabber())
            isHatchGrabberOpen^=true;
        if(control.isChangeDrawSlide())
            isSlideForward^=true;
        hatchGrabber.set(!isHatchGrabberOpen);
        slide.set(getLevel()!=1 && isSlideForward);
        hud.messages[2]=(isSlideForward? "--":"")+(isHatchGrabberOpen? "<":">");
        SmartDashboard.putNumber("armPos",armMaster.getSelectedSensorPosition());
        // SmartDashboard.putNumber("armVel",armMaster.getSelectedSensorVelocity());
        double override=control.getOverrideArm();
        int level=control.getArmPosition();
        if(level!=-1){
            // System.out.printf("move to level %d\n", level);
            armMaster.set(ControlMode.Position, Constants.ARM_POSITION[level]);
            isOverrideArm=false;
        } else if(Math.abs(override)>0.01){
            isOverrideArm=true;
            armMaster.set(ControlMode.PercentOutput, override);
        } else if(isOverrideArm)
            armMaster.set(ControlMode.PercentOutput, 0);
    }

    //TODO
    public int getLevel(){
        int pos=armMaster.getSelectedSensorPosition();
        if(pos<(Constants.ARM_POSITION[0]+Constants.ARM_POSITION[1])/2)
            return 0;
        else if(pos<(Constants.ARM_POSITION[1]+3*Constants.ARM_POSITION[2])/4)
            return 1;
        else return 2;
    }

    private void configTalon(WPI_TalonSRX talon){
        talon.configFactoryDefault();
        talon.setNeutralMode(NeutralMode.Brake);
        talon.configNominalOutputForward(0);
        talon.configPeakOutputForward(1);
        talon.configPeakOutputReverse(-1);
    }

    private void configMaster(WPI_TalonSRX master) {
        master.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        master.config_kP(Constants.ARM_POSITION_SLOT, Constants.ARM_POSITION_GAINS.kP);
		master.config_kI(Constants.ARM_POSITION_SLOT, Constants.ARM_POSITION_GAINS.kI);
		master.config_kD(Constants.ARM_POSITION_SLOT, Constants.ARM_POSITION_GAINS.kD);
		master.config_kF(Constants.ARM_POSITION_SLOT, Constants.ARM_POSITION_GAINS.kF);
		master.config_IntegralZone(Constants.ARM_POSITION_SLOT, Constants.ARM_POSITION_GAINS.kIzone);
		master.configClosedLoopPeakOutput(Constants.ARM_POSITION_SLOT, Constants.ARM_POSITION_GAINS.kPeakOutput);
        master.configAllowableClosedloopError(Constants.ARM_POSITION_SLOT, 0);
        master.selectProfileSlot(Constants.ARM_POSITION_SLOT, 0);

        master.setSensorPhase(false);
        master.configForwardSoftLimitThreshold((int)Constants.ARM_POSITION[2]+1);
        master.configReverseSoftLimitThreshold((int)Constants.ARM_POSITION[0]-20);
        master.configForwardSoftLimitEnable(true);
        master.configReverseSoftLimitEnable(true);
    }
}
