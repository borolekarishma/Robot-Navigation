/*Navigate Class implements the functions that moves the robot from start position to goal using

 * a variation of Hill climbing algorithm called Learning Real Time A* algorithm.
 * 
 * Data about the obstacles, start state and the goal state is read from text file.
 * 
 * JFrame is used to show how the robot progresses towards the goal graphically.
 *  
 *  The path of the robot is shown using the colored lines .
 */
import java.awt.Color;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.Shape;
import java.awt.geom.Line2D;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.Scanner;
import javax.swing.JFrame;

public class Navigate extends JFrame {

	public static Polygon obstacle[];	// Array of polygons in the plane
	public static int start_x, start_y, goal_x, goal_y, st_x, st_y;
	static float start_h, goal_h;
	public static int p, v=0;
	public static Path path[] = new Path[30];	// Array of the index of vertices the robot takes to move from start to goal
	static Scanner r = new Scanner(System.in);
	public static int max_x, max_y;
	
	Navigate()					// Constructor to set the size of the JFrame
	{
		max_x=1000;
		max_y=1000;
		setSize(max_x, max_y);
	}
	
	// Draw all the polygons and the path robot follows towards goal state
	public void paint(Graphics g)		
	{
		g.drawOval(st_x, st_y, 5, 5);
		g.drawString("S", st_x+10, st_y+10);
		g.drawOval(goal_x, goal_y, 5, 5);
		g.drawString("G", goal_x+10, goal_y+10);
		for(int i=0;i<p; i++)
		{
				
				g.drawPolygon(obstacle[i].vertices_x, obstacle[i].vertices_y, obstacle[i].n);	
				g.setColor(Color.cyan);
				g.fillPolygon(obstacle[i].vertices_x, obstacle[i].vertices_y, obstacle[i].n);
		}
		g.setColor(Color.RED);
		if(v!=0)
		{
			g.drawLine(st_x, st_y, obstacle[path[0].index_ob].vertices_x[path[0].index_ver], obstacle[path[0].index_ob].vertices_y[path[0].index_ver]);
			for(int i=0;i<v-1;i++)
			{	
				g.drawLine(obstacle[path[i].index_ob].vertices_x[path[i].index_ver], obstacle[path[i].index_ob].vertices_y[path[i].index_ver], obstacle[path[i+1].index_ob].vertices_x[path[i+1].index_ver], obstacle[path[i+1].index_ob].vertices_y[path[i+1].index_ver]);
			}		
		g.drawLine( obstacle[path[v-1].index_ob].vertices_x[path[v-1].index_ver], obstacle[path[v-1].index_ob].vertices_y[path[v-1].index_ver], goal_x, goal_y);
		}
		else
		{
			g.drawLine(start_x, start_y, goal_x, goal_y);
		}
	}
	
	/*
	 * Reads the data about the start state, goal state and the obstacles
	 * from the text file.
	 * Sets the Jframe visibility true.
	 */
	public static void main(String[] args) throws IOException {
		JFrame f= new Navigate();
		try
		{
			System.out.println("Enter the input file: ");
			String fp=r.next();
			
			File textFile = new File(fp);	//Reads the file "input.txt" that contains the data.
			Scanner sc = new Scanner(textFile);
			
				start_x=sc.nextInt();				//x-coordinate for start state 
				st_x = start_x;
				start_y=sc.nextInt();				//y-coordinate for start state
				st_y = start_y;
				sc.nextLine();
				goal_x=sc.nextInt();				//x-coordinate for goal state
				goal_y=sc.nextInt();				//y-coordinate for goal state
				sc.nextLine();
				p=sc.nextInt();						// accepts the number of obstacles(polygon) in the plane
			
				int i=0;
				sc.nextLine();
				obstacle = new Polygon[p];

				while(i<p)
				{
					obstacle[i] = new Polygon();
					int n = sc.nextInt();			// accepts the number of vertices for each polygon
					obstacle[i].n= n;
					obstacle[i].vertices_x = new int[n];
					obstacle[i].vertices_y = new int[n];
					obstacle[i].vertices_h = new float[n];
					int j=0;
					while(j<n)
					{
						obstacle[i].vertices_x[j]=sc.nextInt();
						obstacle[i].vertices_y[j]=sc.nextInt();
						j++;
					}
					i++;
				}    
		}
		catch(FileNotFoundException e){				
			System.out.println(e);
		}
		
		double sq;
		float h;
		sq= (Math.pow((start_x - goal_x),2))+(Math.pow((start_y - goal_y),2));
		start_h=(float) Math.sqrt(sq);				// calculates the heuristic for start state
		goal_h=0;
		f.setVisible(true);
		getvisible();
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	/*
	 * getvisible function looks for the vertices that are visible to the robot from its current position.
	 * It checks the heuristic of each vertex that is visible to the robot and selects the one with minimum
	 * distance to goal state.
	 * if two visible nodes has equal heuristic it will select the vertex with minimum distance from its current 
	 * position to the node.
	 * It updates the heuristic of the previously visited node for its future reference.
	 */
	private static void getvisible() {
		
		boolean isvisible, isgoal;
		double sq1;
		double sq2;
		int cur_o = 0, cur_v = 0;
		float dist1, dist2, cur_dist = 0, cur_h=start_h;
		
		while(start_h!=0)
		{
			isgoal =checkgoal();
			if(!isgoal)
			{
			for(int i=0;i<p;i++)
			{	
				for(int j =0; j < obstacle[i].n; j++)
				{
					if((start_x != obstacle[i].vertices_x[j]) || (start_y != obstacle[i].vertices_y[j]))
					{
					isvisible = check_pair( i, j);
					if(v!=0)
					{
			
						int index=path[v-1].index_ob;
						if(index==i)
						{
							
							int mid_x = (obstacle[i].vertices_x[j] + start_x)/2;
							int mid_y = (obstacle[i].vertices_y[j] + start_x)/2;
				
							boolean visible1= false;
							int   k, l=obstacle[i].n-1;
				  							
							for (k=0;k<l;k++) 
							{
								if ((obstacle[i].vertices_y[k]> mid_y) != (obstacle[i].vertices_y[l] > mid_y)  && (mid_x < (obstacle[i].vertices_x[l] -  obstacle[i].vertices_x[k]) * (mid_y - obstacle[i].vertices_y[k]) / ((obstacle[i].vertices_y[l]) - (obstacle[i].vertices_y[k])) + obstacle[i].vertices_x[k]))
								{
									visible1=!visible1;
				    	
									l=k;
								}
								if(visible1)
								{
									isvisible = false;
								}

							}
						}
		
					}

					if(isvisible)
					{
												
						obstacle[i].vertices_h[j] = calc(i,j);
						if(cur_h > obstacle[i].vertices_h[j] )
						{
							cur_o=i;
							cur_v=j;
							cur_h=obstacle[i].vertices_h[j];
							sq2= (Math.pow((start_x - obstacle[cur_o].vertices_x[cur_v]),2))+(Math.pow((start_y - obstacle[cur_o].vertices_y[cur_v]),2));
							cur_dist = (float) Math.sqrt(sq2);
						}
						else if(cur_h == obstacle[i].vertices_h[j])
						{
							sq1= (Math.pow((start_x - obstacle[i].vertices_x[j]),2))+(Math.pow((start_y - obstacle[i].vertices_y[j]),2));
							dist1 = (float) Math.sqrt(sq1);
							sq2= (Math.pow((start_x - obstacle[cur_o].vertices_x[cur_v]),2))+(Math.pow((start_y - obstacle[cur_o].vertices_y[cur_v]),2));
							dist2 = (float) Math.sqrt(sq2);
							if(dist1<=dist2)
							{
								cur_o=i;
								cur_v=j;
								cur_h=obstacle[i].vertices_h[j];
								cur_dist= dist1;
							}
						}
					}
					}
				}
			}
			
			if(v!=0)
			{
				int o,ver;
				o=path[v-1].index_ob;
				ver=path[v-1].index_ver;
				obstacle[o].vertices_h[ver] = obstacle[o].vertices_h[ver] + cur_dist;
			}
			
			path[v]=new Path();
			path[v].index_ob = cur_o;
			path[v].index_ver = cur_v;
	
			start_x = obstacle[cur_o].vertices_x[cur_v];
			start_y = obstacle[cur_o].vertices_y[cur_v];
			start_h = cur_h;
			v++;
			}
			else
			{
				System.out.println("Goal reached..");
				start_h = 0;
				break;
			}			
			System.out.println("polygon: " + cur_o + " vertex: " + cur_v);
		}
}
	
	/*
	 * This method checks if the goal is visible to the robot.
	 */
	private static boolean checkgoal() {
		// TODO Auto-generated method stub
		int m,n;
		boolean visible;
		for(int i=0;i<p;i++)
		{
			for(int j=0;j<obstacle[i].n;j++)
			{
				m=j;
				if(j==obstacle[i].n-1)
				{
					n=0;
				}
				else
				{
					n=j+1;
				}
				if((start_x==obstacle[i].vertices_x[m] && start_y==obstacle[i].vertices_y[m])||(start_x==obstacle[i].vertices_x[n] && start_y==obstacle[i].vertices_y[n]))
				{
					visible=false;
				}
				else
					{
					visible = Line2D.linesIntersect(start_x,start_y,goal_x,goal_y,obstacle[i].vertices_x[m],obstacle[i].vertices_y[m],obstacle[i].vertices_x[n],obstacle[i].vertices_y[n]);
					
					}	
				if(visible)
					return false;
			}
		}
		return true;
	}

	/*
	 * The calc function calculates the heuristic of the vertex.
	 * Heuristic is the distance from the vertex to the goal. 
	 */
	private static float calc(int i, int j) {
		float sq, h;
		sq= ((obstacle[i].vertices_x[j] - goal_x)*(obstacle[i].vertices_x[j] - goal_x))+((obstacle[i].vertices_y[j] - goal_y)*(obstacle[i].vertices_y[j] - goal_y));
		h=(float) Math.sqrt(sq);
		return h;
	}

	/*
	 * This function considers a vertex and checks if the vertex is visible to the robot.
	 * It checks if the line connecting the robot's current position and the selected vertex 
	 * is cut by any other side of another polygon.
	 */
	public static boolean check_pair(int a, int b)
	{
		int m,n;
		boolean visible;
		for(int i=0;i<p;i++)
		{
			for(int j=0;j<obstacle[i].n;j++)
			{

				m=j;
				if(j==obstacle[i].n-1)
				{
					n=0;
				}
				else
				{
					n=j+1;
				}
				if((obstacle[a].vertices_x[b]==obstacle[i].vertices_x[m] && obstacle[a].vertices_y[b]==obstacle[i].vertices_y[m])||(obstacle[a].vertices_x[b]==obstacle[i].vertices_x[n] && obstacle[a].vertices_y[b]==obstacle[i].vertices_y[n]) || (start_x==obstacle[i].vertices_x[m] && start_y==obstacle[i].vertices_y[m])||(start_x==obstacle[i].vertices_x[n] && start_y==obstacle[i].vertices_y[n]))
				{
								
					visible=false;
				}
				else
					{
					
					visible = Line2D.linesIntersect(start_x,start_y,obstacle[a].vertices_x[b],obstacle[a].vertices_y[b],obstacle[i].vertices_x[m],obstacle[i].vertices_y[m],obstacle[i].vertices_x[n],obstacle[i].vertices_y[n]);
					
					}
				
				if(visible)
					return false;
			}	
		}
		return true;
	}		
}
