import java.awt.Point;
import java.util.ArrayList;
import java.util.Random;

import geometry.IntPoint;
import dataStructures.RRNode;
import dataStructures.RRTree;

/**
 * class holds information about obstacles  
 * @author Can Eldem
 * @version 1.0
 * 
 */

public class Rrt {

	IntPoint goal_point;  
	IntPoint starting_point;
	int radius_goal_range;  //determines target area 
	int radius; //step size
	double distance_from_start; //distance between initial point and target point 
	int sample_range; // sapmle range for bias 

	public Rrt(IntPoint starting_point,IntPoint goal_point,int radius_goal_range,int radius) {
		super();
		this.goal_point = goal_point;
		this.radius_goal_range = radius_goal_range;
		this.starting_point=starting_point;
		this.radius=radius;
		distance_from_start=distance_between(new Point(starting_point.x,starting_point.y),new Point(goal_point.x,goal_point.y));
		sample_range=Math.max(goal_point.x,goal_point.y)+(Math.max(goal_point.x,goal_point.y)/100)*10;
	}

	public Rrt() {
		super();

	}

	/**
	 * calculates distance between two coordinate.
	 * @param   two points to be calculated 
	 * @return distance
	 */
	double distance_between(Point a, Point b){

		double distance = Math.abs(Math.sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)));  
		return distance;
	}

	/**
	 * creates random point to direct 
	 * @param   none 
	 * @return  sample point 
	 */
	public IntPoint createRandomPoint(){
		int min = Math.max(starting_point.x,starting_point.y);

		Random randomGenerator = new Random();
		int x = randomGenerator.nextInt(sample_range*2)-sample_range; // generate (+) and (-) points up to target point
		int y=  randomGenerator.nextInt(sample_range*2)-sample_range;// generate (+) and (-) points up to target point
		double possible_distance=distance_between(new Point(x,y),new Point(goal_point.x,goal_point.y)); //calculate distance with target 
        double extra=(distance_from_start/100)*10;  // give bias to extra space in case of big obstacles in front of initial point
		
		while(possible_distance>distance_from_start+extra){ //if sample point far away from initial there is not mean for directing that point 
			randomGenerator = new Random();
			x = randomGenerator.nextInt(sample_range*2)-sample_range; // generate (+) and (-) points up to target point
			y=  randomGenerator.nextInt(sample_range*2)-sample_range; // generate (+) and (-) points up to target point
			possible_distance=distance_between(new Point(x,y),new Point(goal_point.x,goal_point.y));
		}

		IntPoint point =new IntPoint(x,y); 
		return point;
	} // createRandomPoint()

	/**
	 * find point which tree going to expand itself 
	 * @param   nearest node sample point current point 
	 * @return  sample point 
	 */
	public IntPoint findDrectionPoint(IntPoint near,IntPoint a,IntPoint b){

		IntPoint target_point=new IntPoint();
		int delta_y= b.y-a.y;
		int delta_x=b.x-a.x;
		// target point should between sample and current point of an robot 
		double angle_A_B=Math.atan2(delta_y, delta_x)*180/Math.PI;
		target_point.x= near.x+(int) (radius*Math.cos(Math.toRadians(angle_A_B)));
		target_point.y= near.y+(int) (radius*Math.sin(Math.toRadians(angle_A_B)));

		return target_point;
	} //intpoint

	/**
	 * calculates weather is given point inside in that point.
	 * @param   query point target point and radius of query point 
	 * @return  true(means inside) or false
	 */
	boolean  insideArea(IntPoint query_point,IntPoint target_point,int query_radius){

		int a=query_point.x-target_point.x;
		int b=query_point.y-target_point.y;
		double number=(a*a)+(b*b);
		double limit=query_radius*query_radius;

		if (number<=limit) {
			return true;
		} 
		else{
			return false ;
		}
	}//inside area

}
