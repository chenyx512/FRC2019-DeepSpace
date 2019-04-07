package frc.robot.chenyxVision.libs;

public class Pose extends Pt implements Interpolable<Pose> {
    public double theta;
    /**
     * @param _theta in degrees
     */
	public Pose(double _x,double _y,double _theta) {
		super(_x, _y);
		theta=_theta;
	}

    public Pose(Vector v, double _theta){
        super(v);
        theta=_theta;
    }

	//TODO can change to constant curvature interpolation
	@Override
	public Pose interpolate(Pose other, double scale) {
		double dx=other.x-x;
		double dy=other.y-y;
		double dtheta=other.theta-theta;
		return new Pose(x+dx*scale, y+dy*scale, theta+dtheta*scale);
    }
    
    public double cosTheta(){
        return Math.cos(Math.toRadians(theta));
    }
    public double sinTheta() {
        return Math.sin(Math.toRadians(theta));
    }

    public Pose plus(Pose other){
        return new Pose(super.plus(other), theta+other.theta);
    }

    @Override
    public Pose scale(double scale){
        return new Pose(super.scale(scale), theta*scale);
    }

    @Override
    public String toString(){
        return String.format("x:%.1f y:%.1f theta:%.1f", x,y,theta);
    }
}
