import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;
import renderables.RenderableOval;
import renderables.RenderablePoint;
import renderables.RenderablePolyline;
import dataStructures.RRNode;
import dataStructures.RRTree;
import easyGui.EasyGui;
import geometry.IntPoint;

/**
 * The main program creates GUI takes input from user and applies methods according to clicked button 
 * shows the output
 * @author Can Eldem
 * @version 1.0
 * 
 */

public class Map
{
	private final EasyGui gui;  // gui object for displaying
	private final RRTree tree;  // tree for rrt 
	public static int discoverey_size=500; // size of the gui object
	private final int fextFieldId;  // textfield id to get starting position of object 
	private final int fextFieldId1;  // textfield id to get starting position of object 
	private final int fextFieldId2;  // textfield id to get starting position of object 
	private int time_label;
	private int step_label;
	private int unused_steps_label;
	private int warning;
	//and add obstacles to object
	public ArrayList<Circle> obstacles =new ArrayList<Circle>(); // list of obstacles added to map
	IntPoint target_point=null; //goal point
    int goal_area;
	
	public Map()
	{

		// Create a new EasyGui instance with a discoverey size graphics panel.
		gui = new EasyGui(discoverey_size,discoverey_size);
		// Add a label to the cell in row 
		warning=gui.addLabel(0, 0, "point coordinations");
	 
		// Add a text field to the cell in row 1 and column 0. The returned ID is
		// stored in fextFieldId to allow access to the field later on.
		fextFieldId = gui.addTextField(1, 0, "x");   //x coordination
		fextFieldId1 = gui.addTextField(2, 0, "y");  // y coordination
		fextFieldId2 = gui.addTextField(3, 0, "radius(target or obstacle)"); //goal range
        
		// Add a button in row 0 column 1. The button is for activating potential field method
		// when pressed it will call the method called PotentialField in "this"
		gui.addButton(0, 1, "Potential Field", this, "potentialField");

		// Add a button in row 1 column 1. It calls the method called "rrtMethod".
		//this will activate RRT in order to solve the problem
		gui.addButton(1, 1, "RRT", this, "rrtMethod");

		//this button is created to adding obstacle to system with entering 
		//x,y,z whereas x,y positions and z is radious of an obstacle
		gui.addButton(0, 2, "Add Obstacle", this, "addObs");

		// Tell the GUI to draw this tree. Because the tree has only just been created
		// there is nothing to draw yet but there will be later on.
		
		gui.addButton(1,2,"Add Target",this, "addTarget");
		
		tree = new RRTree(Color.orange);
		// Create an RRTree to be used later.
	}

	public void runMap(){
		gui.show(); //makes gui visible
	}

	/**
	 * adds target to gui screen according to given input x,y(positions) r goal size
	 * @param  none
	 * @return none 
	 */
	public void addTarget(){
		
		//getting target position of and robot from gui
		int x =Integer.parseInt(gui.getTextFieldContent(fextFieldId)); // x coordination
		int y=Integer.parseInt(gui.getTextFieldContent(fextFieldId1)); // y coordination
		goal_area=Integer.parseInt(gui.getTextFieldContent(fextFieldId2)); // y coordination
		target_point=new IntPoint(x,y);
		gui.setLabelText(warning,"Target Added");
	}

	public void potentialField(){

		long startTime = System.currentTimeMillis(); //get starting time

		//getting starting position of and robot from gui
		int x =Integer.parseInt(gui.getTextFieldContent(fextFieldId)); // x coordination
		int y=Integer.parseInt(gui.getTextFieldContent(fextFieldId1)); // y coordination
		int r=Integer.parseInt(gui.getTextFieldContent(fextFieldId2)); // radius of target
		
		
		gui.setLabelText(time_label, "");
		gui.setLabelText(step_label,"");
		gui.clearGraphicsPanel();
		drawObstacles();
	 
		
	 	int steps=0; //this variable created in order to observe in how many steps 
		//problem going to solved

		//create potential field method to solve problem with starting position 
		//starting position , scan range ,goal range and number of sensors
		//other information is available in potential field such as goal , robot size ,
		// and robot step size 
		PotentialField pg= new PotentialField(new Points(new Point(x,y),0),30,goal_area,12,new Point(target_point.x,target_point.y));

		//draw target area and target point 
		RenderablePoint target_point = new RenderablePoint(pg.goal_point.x,pg.goal_point.y);
		RenderablePoint start_point = new RenderablePoint(pg.starting_point.current.x,pg.starting_point.current.y);
		RenderableOval target_area = new RenderableOval(pg.goal_point.x,pg.goal_point.y,pg.radious_goal_range*2,pg.radious_goal_range*2);
		start_point.setProperties(Color.BLUE, 10.0f);
		target_point.setProperties(Color.BLUE, 10.0f);
		gui.draw(start_point);
		gui.draw(target_point);
		gui.draw(target_area);
		
		pg.sensorTowards(); //set initial sensor towards target 

		//create RenderablePolyline in order to draw a path
		RenderablePolyline line =new RenderablePolyline();

		//check whether robot inside goal area or not 
		while(pg.insideArea(pg.goal_point,new Point(pg.starting_point.current.x,pg.starting_point.current.y),pg.radious_goal_range)==false){
			ArrayList<Points> poop=pg.getNPointsOnCircle(pg.starting_point,pg.move_range);
			poop=pg.obstacleMove(poop, obstacles);
			RenderablePoint point = new RenderablePoint(pg.starting_point.current.x,pg.starting_point.current.y);
			line.addPoint(point);
			pg.selectBest(poop);
			gui.draw(line);
			gui.update();
			steps++;

		}//while
		long endTime = System.currentTimeMillis(); //get finishing time from system
		time_label=gui.addLabel(2,2, "time in seconds:"+(float)((endTime-startTime)*0.001)); // calculate how long does it took to finish this method and display
		step_label=gui.addLabel(3,2,"it took "+steps+" steps to reach goal");//display number of steps
		gui.update();

	}//potential field action end

	/**
	 * adds obstacle to gui screen according to given input x,y(positions) z radius of obstacle
	 * @param  none
	 * @return none 
	 */
	public void addObs() throws InterruptedException{
		
		//getting starting position of and robot from gui
		int x =Integer.parseInt(gui.getTextFieldContent(fextFieldId)); // x coordination
		int y=Integer.parseInt(gui.getTextFieldContent(fextFieldId1)); // y coordination
		int r=Integer.parseInt(gui.getTextFieldContent(fextFieldId2)); // radius of target
		//create an obstacle and put it on map
		Circle circle =new Circle(new IntPoint(x,y),r);
		obstacles.add(circle);
		circle.drawCircle(gui);
		

	}
	
	/**
	 * draws inserted obstacles
	 * @param  none
	 * @return none 
	 */
	public void drawObstacles(){
		
		for(int i=0;i<obstacles.size();i++){
			//create an obstacle and put it on map
			Circle circle =obstacles.get(i);
			circle.drawCircle(gui);
		}
	}

	public void rrtMethod() throws InterruptedException{
		
		gui.clearGraphicsPanel();
		gui.draw(tree);
		
		long startTime = System.currentTimeMillis();//get starting time
		int steps=0; //this variable created in order to observe in how many steps 

		//getting starting position of and robot from gui
		int x =Integer.parseInt(gui.getTextFieldContent(fextFieldId)); // x coordination
		int y=Integer.parseInt(gui.getTextFieldContent(fextFieldId1)); // y coordination
		int r=Integer.parseInt(gui.getTextFieldContent(fextFieldId2)); // radius of target
		
		//clean previous report information
		gui.setLabelText(time_label, "");
		gui.setLabelText(step_label,"");
		gui.setLabelText(unused_steps_label,"");
		
		drawObstacles(); //draw obstacles
		
		//initialize first values 
		IntPoint start=new IntPoint(x,y);
		RRNode nearest=null;

		tree.setStartAndGoal(start,target_point, goal_area); //initialize first settings
		Rrt  rrtmehod=new Rrt(start,target_point,goal_area,20); //initialize method
		RenderablePolyline  line =new RenderablePolyline(); // drawing line for backtrace

		while(rrtmehod.insideArea(rrtmehod.goal_point,rrtmehod.starting_point,rrtmehod.radius_goal_range)==false){

			IntPoint sample_point=rrtmehod.createRandomPoint(); //get an sample point 
			nearest = tree.getNearestNeighbour(sample_point);// Returns the nearest node to target
			//get and target point to expend 
			IntPoint target_point=rrtmehod.findDrectionPoint(new IntPoint(nearest.x,nearest.y),new IntPoint(start.x,start.y),new IntPoint(sample_point.x,sample_point.y));

			//get obstacles 
			for(int j=0;j<obstacles.size();j++){
				Circle temp=obstacles.get(j);
				//if target point appears inside an obstacle find another target point 
				while(rrtmehod.insideArea(temp.center,target_point,temp.radius)==true){
					sample_point=rrtmehod.createRandomPoint();
					nearest = tree.getNearestNeighbour(sample_point);
					target_point=rrtmehod.findDrectionPoint(new IntPoint(nearest.x,nearest.y),new IntPoint(start.x,start.y),new IntPoint(sample_point.x,sample_point.y));
					steps++;
				}//while		
			}

			tree.addNode(nearest,target_point); //parent 
			steps++; //increase step number 
			rrtmehod.starting_point=target_point; //change current point of robot
			gui.update(); //update gui
		}//while

		//this part for drawing back trace to starting point 
		ArrayList<IntPoint> path=tree.getPathFromRootTo(nearest); //get points to starting point 

		for(int i=0;i<path.size();i++){
			IntPoint temp_path=path.get(i);
			line.addPoint(new RenderablePoint(temp_path.x,temp_path.y));
		}

		line.setProperties(Color.RED, 1.3f);
		gui.draw(line);
		long endTime = System.currentTimeMillis();//get finishing time from system
		time_label=gui.addLabel(2,2, "time in seconds:"+(float)((endTime-startTime)*0.001)); // calculate how long does it took to finish this method and display
		step_label=gui.addLabel(3,2,"it took "+steps+" steps to reach goal");//display number of steps
		unused_steps_label=gui.addLabel(4,2,"unused steps:"+(steps-path.size()));
		gui.update();

	}//rrtmethod


	// MAIN part of program
	public static void main(String[] args)
	{
		Map map = new Map();
		map.runMap();
	}
}
