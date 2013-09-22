import java.awt.Point;
import java.util.ArrayList;

import renderables.RenderableOval;

/**
 * This class contains methods and information for  potential field technique  
 * shows the output
 * @author Can Eldem
 * @version 1.0
 */


public class PotentialField {

	Points starting_point;  //starting point of and robot
	public Point goal_point; //goal position
	int scan_range; // Radius scan range of robot
	int move_range=10; // Radius step range of robot
	int radious_goal_range;  // radius in order to detect goal range of robot
	double potential;  // holds temporary potential information
	int number_of_sensor;  // number of sensors that agent going to use
	double view_angle=180;  //view angle of an robot
	int robot_radius=5;    // size of an robot

	int first_loop_start,first_loop_end,second_loop_start,second_loop_end;
	int division1,division2;

	public PotentialField(Points starting_point,int scan_range,int radious_goal_range,int number_of_sensor,Point goal_Point) {

		this.starting_point = starting_point;
		this.scan_range = scan_range;
		this.radious_goal_range = radious_goal_range;
		this.number_of_sensor = number_of_sensor;
		this.goal_point=goal_Point;
	}

	/**
	 * enables agent to draw sensor circle towards target
	 *  
	 */
	public  void sensorTowards(){
		if((goal_point.x-starting_point.current.x)<0  ){  //means target behind robot
			first_loop_start=90;
			first_loop_end=180;
			second_loop_start=180;
			second_loop_end=270;
			division1=90;
			division2=180;
		}
		else{  //target towards robot
			first_loop_start=0;
			first_loop_end=90;
			second_loop_start=270;
			second_loop_end=360;
			division1=1;
			division2=1;
		}
	}

	/**
	 * Attempts to read the text file specified by filename and returns an array of Strings (paragraphs) found in the file.
	 * @param  get points and range so that it can get points on that range
	 * @return returns calculated points 
	 */
	public ArrayList<Points> getNPointsOnCircle(Points p,int range){

		// calculates number of gaps 
		int check_angle=(int)view_angle/number_of_sensor; 
		ArrayList<Points> p_points =new ArrayList<Points>(); //new points going to be calculated

		for(int i=first_loop_start+p.angle;i<=first_loop_end+p.angle;i=i+check_angle){ //stars a loop considering previous angle for first quarter 
			Point p_temp=new Point();
			p_temp.x= (int) (range*Math.cos(Math.toRadians(i))); //get x position
			p_temp.y= (int) (range*Math.sin(Math.toRadians(i))); //get y position
			p_temp.x=p_temp.x+starting_point.current.x; // add them to current position
			p_temp.y=p_temp.y+starting_point.current.y; // add them to current position
			Points item=new Points(p_temp,(i%division1));
			item.potential=(float)distance_between(goal_point,item.current); //calculate goal distance
			p_points.add(item);
		}//for 

		for(int i=second_loop_start+ p.angle ;i<=second_loop_end+p.angle ;i=i+check_angle){//stars a loop considering previous angle for last quarter

			Point p_temp1=new Point();
			p_temp1.x= (int) (range*Math.cos(Math.toRadians(i)));//get x position
			p_temp1.y= (int) (range*Math.sin(Math.toRadians(i)));//get y position
			p_temp1.x=p_temp1.x+starting_point.current.x;// add them to current position
			p_temp1.y=p_temp1.y+starting_point.current.y;// add them to current position
			Points item1=new Points(p_temp1,(i%division2));
			item1.potential=(float) distance_between(goal_point,item1.current);//calculate goal distance
			p_points.add(item1);
		}//for 

		return p_points;

	}//end of method

	/**
	 * calculates distance between two coordiantion.
	 * @param   two points to be calculated 
	 * @return distance
	 */
	double distance_between(Point a, Point b){

		double distance = Math.abs(Math.sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)));  
		return distance;
	}

	/**
	 * calculates distance between two coordiantion.
	 * @param   two points to be calculated 
	 * @return  true(means intersect) or false 
	 */
	boolean isIntersect(Point a,Point b,int r1,int r2){
		double distance = distance_between(a, b);

		if(r1+r2<distance)
			return false;
		else			
			return true;
	}

	/**
	 * calculates weather is given point inside in that point.
	 * @param   query point target point and radius of query point 
	 * @return  true(means inside) or false
	 */
	boolean  insideArea(Point query_point,Point target_point,int query_radius){

		int a=query_point.x-target_point.x;
		int b=query_point.y-target_point.y;
		double number=(a*a)+(b*b);
		double limit=query_radius*query_radius;

		if (number<=limit) {
			return true;
		} 
		else {
			return false ;
		}
	}//inside area

	/**
	 * gets the minimum potential (obstacle or distance) according to given option
	 * @param   gets points and searching option 0 obstacle potential ,1 distance potential 
	 * @return  min index
	 */
	public int getMinValueIndex(ArrayList<Points> p,int type){  
		double minValue;
		if(type==0)
			minValue = p.get(0).potential;
		else
			minValue = p.get(0).obstacle_porential;

		int index=0;

		for(int i=1;i<p.size();i++){  
			if(type==0){
				if(p.get(i).potential < minValue){  
					index=i;  
					minValue=p.get(i).potential;
				} //if
			}//for

			if(type==1){
				if(p.get(i).obstacle_porential < minValue){  
					index=i;  
					minValue=p.get(i).obstacle_porential;
				}//if
			}//if
		}//for
		return index;
	}// get min value 

	/**
	 * gets points and chooses best direction to move according to potentials and updates current point of an agent 
	 * @param   gets points 
	 * @return  void
	 */
	public void selectBest(ArrayList<Points> p){

		int index=getMinValueIndex(p,0); //gets min goal potential
		int obs_index=getMinValueIndex(p, 1); //gets min obs potential 

		if(index==obs_index)  // if obs potential and goal potential is in same index
			starting_point=p.get(index);
		else{  // if obs potential and goal potential is in different
			if(p.get(index).obstacle_porential==0) //choose point with min obstacle 
				starting_point=p.get(index);
			else
				starting_point=p.get(obs_index);
		}//else
	} // select best

	/**
	 * gets points and check whether they are intersect with obstacle or not if it is obstacle potential will be calculated for this 
	 * @param   gets points 
	 * @return  updated points with obstacle potential
	 */
	public  ArrayList<Points> obstacleMove(ArrayList<Points> p,ArrayList<Circle> obstacles){

		ArrayList<Points> hitpoints = getNPointsOnCircle(starting_point, scan_range); //gets hit points


		for(int j=0;j<obstacles.size();j++){ //get obstacles from map
			Circle temp=obstacles.get(j);

			for(int i=0;i<hitpoints.size();i++){ // checking every hit point
				Points temp_hit_point=hitpoints.get(i); 
				if(isIntersect(new Point(temp.center.x,temp.center.y),temp_hit_point.current, robot_radius, temp.radius)){ //if hit point with robot intersects with obstacle
					double obs_pt=Double.POSITIVE_INFINITY;
					p.get(i).obstacle_porential=obs_pt; //update obstacle sentence

				}//if			

				else{
					double distance_to_obs_center=distance_between(temp_hit_point.current,new Point(temp.center.x,temp.center.y)); //calculate obstacle potential based on distances with robot center and obstacle center
					if(distance_to_obs_center<=(temp.radius+this.scan_range)){
						double obs_pt=(1/Math.pow(distance_to_obs_center,distance_to_obs_center)); // calculate obstacle potential for non touching close hit points
						p.get(i).obstacle_porential=obs_pt; //update obstacle sentence
					}
				}
			}//hit point for
		}//for obstacle
		return p; //return updated points
	}

}
