import java.awt.Point;



public class Points {

	Point current; //position of point
	int angle;   // shows which angle that point took according to coordiante systems 
	double potential; // goal potential 
	double obstacle_porential=0; // obstacle potential 


	public Points() {
		super();
		// TODO Auto-generated constructor stub
	}

	public Points(Point p, int angle) {
		super();
		this.current = p;
		this.angle = angle;
	}


	@Override
	public String toString() {
		return "x:"+current.x+" y:"+current.y+" previous angle:"+angle+" potential:"+potential;
	}



}
