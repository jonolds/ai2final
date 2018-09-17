import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.MouseEvent;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.TreeSet;
import java.util.Vector;

class Agent {
	public Vector<int[]> path = new Vector<int[]>(); //elem 0 is goal. path.get(path.size()-1) is init pos
	int[] bigGoal = new int[] {100, 100};
	PriorityQueue<State> frontier = new PriorityQueue<>();

	void drawPlan(Graphics g, Model m) {
		drawFrontier(g);
		drawPath(g, m);
	}

	void update(Model m) {
		Controller c = m.getController();
		Planner.PathAndFrontier combo = (new Planner(m, bigGoal[0], bigGoal[1])).ucs();
		path = combo.path;
		frontier = combo.frontier;
		if(m.getX() == m.getDestinationX() && m.getY() == m.getDestinationY()) {
			if(path.size() > 0)
				m.setDestination(path.get(path.size()-1)[0], path.get(path.size()-1)[1]);
			else if(isWithinTen(bigGoal[0], bigGoal[1], (int)m.getX(), (int)m.getY()))
				m.setDestination((float)bigGoal[0], (float)bigGoal[1]);
		}
		while(true) {
			MouseEvent e = c.nextMouseEvent();
			if(e == null)
				break;
			bigGoal = new int[] {e.getX(), e.getY()};
		}
	}
	
	boolean isWithinTen(int x1, int y1, int x2, int y2) {
		return Math.abs(x1 - x2) < 10 && Math.abs(y1 - y2) < 10;
	}
	
	void drawPath(Graphics g, Model m) {
			g.setColor(Color.red);
			int[] last = bigGoal;
			for(int i = 0; i < path.size(); i++) {
				g.drawLine(last[0], last[1], path.get(i)[0], path.get(i)[1]);
				last = new int[] {path.get(i)[0], path.get(i)[1]};
			}
			if(!path.isEmpty())
				g.drawLine((int)m.getX(), (int)m.getY(), path.get(path.size()-1)[0], path.get(path.size()-1)[1]);
	}
	
	void drawFrontier(Graphics g) {
		g.setColor(Color.YELLOW);
		for(State s : frontier)
			g.fillOval(s.pos[0], s.pos[1], 10, 10);
	}
	
	public static void main(String[] args) throws Exception {
		Controller.playGame();
	}
	
	
	
	class Planner {
		Model m;
		int[] startPos, goalPos, actionPairs = new int[] {10,-10, 10,0, 10,10, 0,10, 0,-10, -10,-10, -10,0, -10,10};
		
		Planner(Model m, int destX, int destY) {
			this.m = m;
			this.startPos = new int[] {(int)m.getX(), (int)m.getY()};
			this.goalPos = new int[] {destX, destY};
		}

		PathAndFrontier ucs() {
			PriorityQueue<State> frontier = new PriorityQueue<State>(new CostComp()) {{add(new State(0.0, null, startPos));}};
			TreeSet<State> visited = new TreeSet<State>(new PosComp()) {{add(new State(0.0, null, startPos));}};
			while(!frontier.isEmpty()) {
				State s = frontier.poll();
				if(isWithinTen(s.pos[0], s.pos[1], goalPos[0], goalPos[1]))
					return new PathAndFrontier(state2moves(s), frontier);
				for(int i = 0; i < 16; i+=2) {
					int[] newPos = new int[] {s.pos[0] + actionPairs[i], s.pos[1] + actionPairs[i+1]};
					if(newPos[0] >= 0 && newPos[0] < 1200 && newPos[1] >= 0 && newPos[1] < 600) {
						float speed = m.getTravelSpeed((float)newPos[0], (float)newPos[1]);
						double actCost = this.getDistance(s.pos[0], s.pos[1], newPos[0], newPos[1])/speed;
						State newChild = new State(actCost, s, newPos);
						if(visited.contains(newChild)) {
							State oldChild = visited.floor(newChild);
							if(oldChild != null) if (s.cost + actCost < oldChild.cost) {
								oldChild.cost = s.cost + actCost;
								oldChild.parent = s;
							}
						}
						else {
							newChild.cost += s.cost;
							frontier.add(newChild);
							visited.add(newChild);
						}
					}
				}
			}
			return null;
		}
		
		public Vector<int[]> state2moves(State s) {
			Vector<int[]> moves = new Vector<>();
			if(s != null)
				while(s.parent != null) {
					moves.add(new int[]{s.pos[0], s.pos[1]});
					s = s.parent;
				}
			return moves;
		}
		
		public class PathAndFrontier {
			Vector<int[]> path;
			PriorityQueue<State> frontier;
			PathAndFrontier(Vector<int[]> path, PriorityQueue<State> frontier) {
				this.path = path; this.frontier = frontier;
			}
		}
		
		public float getDistance(float x1, float y1, float x2, float y2) {
			return (float)Math.sqrt(Math.pow(x2-x1, 2) + (float)Math.pow(y2-y1, 2));
		}
		
		class CostComp implements Comparator<State> {
			public int compare(State a, State b) {
				return Double.compare(a.cost, b.cost);
			}
		}
		class PosComp implements Comparator<State> {
			public int compare(State a, State b) {
				for(int i = 0; i < 2; i++)
					if(a.pos[i] < b.pos[i])
						return -1;
					else if(a.pos[i] > b.pos[i])
						return 1;
				return 0;
			}
		}
	}
	
	class State {
		double cost;
		State parent;
		int[] pos;

		State(double cost, State parent, int[] pos) {
			this.cost = cost;
			this.parent = parent;
			this.pos = pos;
		}
	}
}