import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.*;


public class MtStHelensExp_915107182 implements AIModule
{
    private double getHeuristic(final TerrainMap map, final Point pt1, final Point pt2)
    {
        double hvalue, hvalueplus;
        //double slop;
        //int optinal_move = Math.max(Math.abs(pt2.x - map.getStartPoint().x), Math.abs(pt2.y - map.getStartPoint().y));
        //slop = Math.abs(map.getTile(map.getStartPoint()) - map.getTile(pt2)) / optinal_move;

        int basicmove = Math.max(Math.abs(pt2.x - pt1.x), Math.abs(pt2.y - pt1.y));
        //hvalueplus = basicmove * Math.pow(2.0, ((optinal_move - basicmove) * slop - map.getTile(pt1)) / basicmove);
        double height_difference = map.getTile(pt2) - map.getTile(pt1);
        if (height_difference != 0) {
            hvalue = basicmove * Math.pow(2.0, (height_difference / basicmove));
        } else {
            hvalue = basicmove;
        }
        return hvalue;
    }

    public class PointPro {
        public Point point;
        public double fvalue;

        public PointPro(final Point point, double fvalue) {
            this.point = point;
            this.fvalue = fvalue;
        }

    }

    public class PointProComparator implements Comparator<PointPro> {
        public int compare(PointPro p1, PointPro p2) {
            double f1, f2;
            f1 = p1.fvalue;
            f2 = p2.fvalue;
            if (f1 > f2) {
                return 1;
            } else {
                return 0;
            }
        }
    }

    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map)
    {
        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();

        // Keep track of where we are and add the start point.
        int optinal_move = Math.max(Math.abs(map.getEndPoint().x - map.getStartPoint().x), Math.abs(map.getEndPoint().y - map.getStartPoint().y));
        PointPro CurrentPoint = new PointPro(map.getStartPoint(), getHeuristic(map, map.getStartPoint(), map.getEndPoint()));

        double slop;
        slop = 1.0 * (map.getEndPoint().y - map.getStartPoint().y) / (map.getEndPoint().x - map.getStartPoint().x);

        //creat hashmap to store the g() for each point and the point it from.
        HashMap<Point, Double> GValues = new HashMap<Point, Double>();
        //creat hashmap to store the path point of each point.
        HashMap<Point, Point> lastpoint = new HashMap<Point, Point>();
        //creat a pq to store get the most optimal point when using A star.
        PriorityQueue<PointPro> Pointpq = new PriorityQueue<PointPro>(200, new PointProComparator());

        //initialize maps and the pq by start point
        lastpoint.put(map.getStartPoint(), null);
        GValues.put(map.getStartPoint(), 0.00);
        Pointpq.add(CurrentPoint);

        int times = 0;
        while (Pointpq.size() != 0) {
            CurrentPoint = Pointpq.remove();
            if (CurrentPoint.point.x < map.getStartPoint().x || CurrentPoint.point.y < map.getStartPoint().y) {
                continue;
            }

            if (times > optinal_move) {
                if (CurrentPoint.point.equals(map.getEndPoint())) {
                    break;
                }
            }
            for (Point surroundingpoint : map.getNeighbors(CurrentPoint.point)) {
                double newgvalue = GValues.get(CurrentPoint.point) + map.getCost(CurrentPoint.point, surroundingpoint);
                if (!GValues.containsKey(surroundingpoint) || newgvalue < GValues.get(surroundingpoint)){
                    GValues.put(surroundingpoint, newgvalue);
                    lastpoint.put(surroundingpoint, CurrentPoint.point);
                    Pointpq.add(new PointPro(surroundingpoint, newgvalue + getHeuristic(map, surroundingpoint, map.getEndPoint())));
                }
            }
            times++;
        }

        //build the path base on the last point map we create.
        Point tracebackpoint = new Point(map.getEndPoint());
        path.add(tracebackpoint);
        while (!tracebackpoint.equals(map.getStartPoint())) {
            tracebackpoint = new Point(lastpoint.get(tracebackpoint));
            path.add(0, tracebackpoint);
        }

        // We're done!  Hand it back.
        return path;
    }
}
