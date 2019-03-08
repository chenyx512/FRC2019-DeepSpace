package frc.robot.trajectory;

import frc.robot.Constants;
import frc.robot.libs.*;

public class Kinematics {
    public static final double MAX_LINEAR_V = 1300; // mm * s^-1
    public static final double MAX_ANGULAR_V = Math.PI; // rad * s^-1
    public static final double MAX_LINEAR_A = 500; // mm * s^-2
    public static final double MAX_ANGULAR_A = Math.PI / 2; // rad * s^-2

    /* based on the maximum speed of a single side motor
     * eg: when going forward while turning right, this makes sure the left motors velocity stays below limit
     */
    public static double getMaxSpeed(double K1) {
        return MAX_LINEAR_V/(1+Constants.DRIVE_WIDTH*Math.abs(K1)/2);
    }

    /*
     * a+alpha * width/2 < max_a
     * alpha=a*K + v*v*dK/ds
     */
    public static double getMaxAcc(double K, double v, double dKds) {
        K=Math.abs(K);
        dKds=Math.abs(dKds);
        double RV=(MAX_LINEAR_A-v*v*dKds*Constants.DRIVE_WIDTH/2) / (1+K*Constants.DRIVE_WIDTH/2);
        //System.out.printf("k=%.8f v=%.2f dkds=%.9f output=%.5f\n",K,v,dKds, RV);
        return RV;
    }

    /**
     * Vector.x is left and Vector.y is right
     * 
     * @param omiga in radians and counterclockwise positive
     */
    public static Vector getWheelDynamics(double v, double omiga) {
        if (Math.abs(omiga) < 1e-5)
            return new Vector(v, v);
        double deltaV = Constants.DRIVE_WIDTH * omiga / (2 * Constants.TRACK_SCRUB_FACTOR);
        // System.out.printf("v %.1f omiga %.1f deltaV %.1f\n", v, omiga, deltaV);
        return new Vector(v - deltaV, v + deltaV);
    }
}