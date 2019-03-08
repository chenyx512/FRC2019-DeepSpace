package frc.robot.trajectory;

import frc.robot.Robot;
import frc.robot.libs.*;

public class Trajectory {
    public static final int ITERATION_NUM = 150; // TODO tune this
    public static final double ITERATION_DT = 1.0/ITERATION_NUM;

    public QuinticSpline spline;
    private MotionProfile[] profile;
    public double time;
    public InterpolatingTreeMap2<InterpolatingDouble, MotionPose> profileMap;

    public Trajectory(QuinticSpline _spline, double startV, double endV) {
        profile = new MotionProfile[ITERATION_NUM + 1];
        for (int i = 0; i <= ITERATION_NUM; i++)
            profile[i] = new MotionProfile();
        spline = _spline;
        generateTrajectory(startV, endV);
        System.out.printf("\nend generating profile %.3f\n", Robot.time.get());
        profileMap = new InterpolatingTreeMap2<InterpolatingDouble, MotionPose>(ITERATION_NUM + 1);
        for (int i = 0; i <= ITERATION_NUM; i++)
            profileMap.put(new InterpolatingDouble(profile[i].time), new MotionPose(
                    spline.getPoint((double) i / ITERATION_NUM).x, spline.getPoint((double) i / ITERATION_NUM).y,
                    spline.getHeading((double) i / ITERATION_NUM), profile[i].v, profile[i].a, profile[i].curvature,
                    spline.getDCurvatureDs((double) i / ITERATION_NUM), profile[i].distance, profile[i].time));
        time = profile[ITERATION_NUM].time - 1e-5;
    }

    public Trajectory(QuinticSpline _spline) {
        this(_spline, 0.0, 0.0);
    }

    public void generateTrajectory(double startV, double endV) {

        // forward pass
        profile[0].curvature = spline.getCurvature(0);
        profile[0].a = Kinematics.getMaxAcc(profile[0].curvature, startV, spline.getDCurvatureDs(0));
        profile[0].v = startV;
        for (int i = 1; i <= ITERATION_NUM; i++) {
            double t = (double) i / ITERATION_NUM;
            profile[i].curvature = spline.getCurvature(t);
            // based on previous
            profile[i].distance = profile[i - 1].distance + spline.getVelocity(t - ITERATION_DT) * ITERATION_DT; // correct
            profile[i].v = Math.sqrt(profile[i - 1].v * profile[i - 1].v  + 2 * profile[i - 1].a * (profile[i].distance - profile[i - 1].distance));
            profile[i].v = Math.min(profile[i].v, Kinematics.getMaxSpeed(spline.getCurvature(t)));
            // apply constraints
            profile[i].a = Kinematics.getMaxAcc(profile[i].curvature, profile[i].v, spline.getDCurvatureDs(t));
            // System.out.printf("at t=%.3f a=%.1f K=%.1f dkds=%.1f\n", t, profile[i].a, profile[i].curvature,
            //         spline.getDCurvatureDs(t));
            // if (profile[i].a<0) System.out.printf("initial negative a=%.1f at
            // t=%.3f",profile[i].a,t);
        }

        // backward pass
        profile[ITERATION_NUM].v = endV;
        profile[ITERATION_NUM].a = Kinematics.getMaxAcc(profile[ITERATION_NUM].curvature, profile[ITERATION_NUM].v,
                spline.getDCurvatureDs(1));
        for (int i = ITERATION_NUM - 1; i > 0; i--) {
            double t = (double) i / ITERATION_NUM;
            profile[i].v = Math.min(profile[i].v, Math.sqrt(profile[i+1].v*profile[i+1].v+2*profile[i+1].a*(profile[i+1].distance-profile[i].distance)));
            // update max_a as v goes lower
            profile[i].a = Kinematics.getMaxAcc(profile[i].curvature, profile[i].v, spline.getDCurvatureDs(t));
        }
        // add time/a to the trajectory
        profile[0].a = Math.sqrt(profile[1].v * profile[1].v - profile[0].v-profile[0].v / 2 / profile[1].distance);
        for (int i = 1; i < ITERATION_NUM; i++) {
            double ds = profile[i].distance - profile[i - 1].distance;
            profile[i].time = profile[i - 1].time + ds / (profile[i - 1].v + profile[i].v) * 2;
            profile[i].a = (profile[i + 1].v * profile[i + 1].v - profile[i - 1].v * profile[i - 1].v) / 2
                    / (profile[i + 1].distance - profile[i - 1].distance);
        }
        profile[ITERATION_NUM].time = profile[ITERATION_NUM - 1].time
                + (profile[ITERATION_NUM].distance - profile[ITERATION_NUM - 1].distance)
                        / profile[ITERATION_NUM - 1].v;
        profile[ITERATION_NUM].a = (profile[ITERATION_NUM].v * profile[ITERATION_NUM].v - profile[ITERATION_NUM - 1].v * profile[ITERATION_NUM - 1].v)
                     / 2 / (profile[ITERATION_NUM].distance - profile[ITERATION_NUM - 1].distance);
        // for (int i = 0; i <= ITERATION_NUM; i++) {
        //     System.out.printf("i:%d time:%.2f dis:%.2f v:%.2f a:%.2f\n", i, profile[i].time, profile[i].distance,
        //             profile[i].v, profile[i].a);
        // }
    }

    public class MotionProfile {
        public double v, a; // alpha omiga
        public double curvature, distance, time;

        public MotionProfile() {
            v = a = 0;
            curvature = distance = time = 0;
        }

        public double getAlpha() {
            return a * curvature;
        }

        public double getOmiga() {
            return v * curvature;
        }
    }
}
