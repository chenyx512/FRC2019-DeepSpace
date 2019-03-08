package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.DriveTrain;

/**
 * x,y set up as math, unit is milimeter, theta in degree, omiga means omiga-z
 * in deg/sec
 */
public class Position {
    public double x, y, theta;
    public double lastTheta;
    public double vx, vy, omiga;

    private Encoder mEncoderFL, mEncoderFR;
    private PigeonIMU mPigeon;

    public Position() {
        mPigeon = new PigeonIMU(DriveTrain.RL);

        mEncoderFL = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        mEncoderFL.setDistancePerPulse(-1.89);
        mEncoderFR = new Encoder(3, 4, false, Encoder.EncodingType.k4X);
        mEncoderFR.setDistancePerPulse(1.89);

        reset();
    }

    public void reset() {
        mPigeon.setYaw(0);
        mEncoderFL.reset();
        mEncoderFR.reset();
        x = y = 0;
        theta = lastTheta = 90;
    }

    public void update() {
        double ypr[] = new double[3];
        mPigeon.getYawPitchRoll(ypr);
        lastTheta = theta;
        theta = ypr[0] + 90;
        omiga = (theta - lastTheta) / 0.02;
        // System.out.printf("l:%.2f
        // r:%.2f\n",mEncoderFL.getRate(),mEncoderFR.getRate());
        // according to robot
        double rVy = (mEncoderFL.getRate() + mEncoderFR.getRate()) / 2;
        double rVx = mEncoderFR.getRate() - rVy
                + (Constants.kRobotHalfWidth + Constants.kRobotHalfLength) * omiga / 180 * 3.1415926;
        // if(Robot.cnt%10==0)
        //     System.out.printf("rvx:%.1f rvy:%.1f lEncoder:%.1f rEncoder:%.1f debug:%.1f\n",rVx,rVy,mEncoderFL.getRate(),mEncoderFR.getRate(),
        //             (Constants.kRobotHalfWidth + Constants.kRobotHalfLength) * omiga / 180 * 3.1415926);
        vx = Math.cos(Math.toRadians(theta - 90)) * rVx + Math.cos(Math.toRadians(theta)) * rVy;
        vy = Math.sin(Math.toRadians(theta - 90)) * rVx + Math.sin(Math.toRadians(theta)) * rVy;
        x += vx * 0.02;
        y += vy * 0.02;
    }
}