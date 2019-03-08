package frc.robot.trajectory;
import frc.robot.libs.Pose;

public class QuinticSpline {
	private double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
    private double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;

    public QuinticSpline(Pose p0, Pose p1) {
        double scale = 1.2 * p0.distance(p1);
        x0 = p0.x;
        x1 = p1.x;
        dx0 = p0.cosTheta() * scale;
        dx1 = p1.cosTheta() * scale;
        ddx0 = 0;
        ddx1 = 0;
        y0 = p0.y;
        y1 = p1.y;
        dy0 = p0.sinTheta() * scale;
        dy1 = p1.sinTheta() * scale;
        ddy0 = 0;
        ddy1 = 0;

        computeCoefficients();
    }

    private void computeCoefficients() {
        ax = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        bx = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        cx = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        dx = 0.5 * ddx0;
        ex = dx0;
        fx = x0;

        ay = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
        by = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
        cy = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
        dy = 0.5 * ddy0;
        ey = dy0;
        fy = y0;
    }
    
    public Pose getPoint(double t) {
        double x = ax * t * t * t * t * t + bx * t * t * t * t + cx * t * t * t + dx * t * t + ex * t + fx;
        double y = ay * t * t * t * t * t + by * t * t * t * t + cy * t * t * t + dy * t * t + ey * t + fy;
        return new Pose(x, y, getHeading(t));
    }
    
    public double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }
    /* Curvature is 1/r
     * positive curvature turns left (right hand rule)
     */
    public double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t)) / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy
                (t) * dy(t))));
    }

    private double getDCurvatureDt(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }
    /** if there is more curvature RV is positive */
    public double getDCurvatureDs(double t) {
        double RV=getDCurvatureDt(t)/getVelocity(t);
        return getCurvature(t)*RV>0? Math.abs(RV):-Math.abs(RV);
    }
    
    /* return the angle in degree
     */
    public double getHeading(double t) {
        return Math.toDegrees(Math.atan2(dy(t), dx(t)));
    }
    
    private double dx(double t) {
        return 5 * ax * t * t * t * t + 4 * bx * t * t * t + 3 * cx * t * t + 2 * dx * t + ex;
    }
    private double dy(double t) {
        return 5 * ay * t * t * t * t + 4 * by * t * t * t + 3 * cy * t * t + 2 * dy * t + ey;
    }
    private double ddx(double t) {
        return 20 * ax * t * t * t + 12 * bx * t * t + 6 * cx * t + 2 * dx;
    }
    private double ddy(double t) {
        return 20 * ay * t * t * t + 12 * by * t * t + 6 * cy * t + 2 * dy;
    }

    private double dddy(double t) {
        return 60 * ay * t * t + 24 * by * t + 6 * cy;
    }

    private double dddx(double t) {
        return 60 * ax * t * t + 24 * bx * t + 6 * cx;
    }
}