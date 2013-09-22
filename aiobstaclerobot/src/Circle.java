import java.awt.Color;

import easyGui.EasyGui;
import renderables.RenderableOval;
import geometry.IntPoint;

/**
 * class holds information about obstacles  
 * @author Can Eldem
 * @version 1.0
 * 
 */

public class Circle {

	IntPoint center; //center coordiantions 
	int radius;   //radious of obstacle 
	RenderableOval circle; // Renderable object in order to draw on gui
	
	public Circle(IntPoint center, int radius) {
		super();
		this.center = center;
		this.radius = radius;
	}
	
	/**
	 * draws a circle on a gui
	 * @param  get gui reference in order to draw
	 * @return none 
	 */
	public void drawCircle(EasyGui gui){
		
	    circle = new RenderableOval(center.x, center.y,radius*2,radius*2);
		circle.setProperties(Color.RED, 1.0f, true);
		gui.draw(circle);
		gui.update();
		 
	}
	
}
