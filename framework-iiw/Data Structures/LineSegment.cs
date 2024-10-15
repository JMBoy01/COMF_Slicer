using Clipper2Lib;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Media3D;

namespace framework_iiw.Data_Structures
{
    internal class LineSegment
    {
        private List<Point3D> pointsOnLine;

        public LineSegment()
        {
            pointsOnLine = new List<Point3D>();
        }
        
        public LineSegment(Point3D point) { 
            pointsOnLine = new List<Point3D>() { point };
        }

        public LineSegment(List<Point3D> points)
        {
            this.pointsOnLine = points;
        }

        public Point3D GetLineStart()
        {
            return pointsOnLine[0];
        }

        public Point3D GetLineEnd()
        {
            return pointsOnLine[^1];
        }
    
        public void AddPoint(Point3D point)
        {
            pointsOnLine.Add(point);
        }
    
        public bool IsValid()
        {
            return pointsOnLine.Count == 2 && pointsOnLine[0] != pointsOnLine[1];
        }
    
        public PathD ToPathD()
        {
            var pointA = new PointD { x = GetLineStart().X, y = GetLineStart().Y };
            var pointB = new PointD { x = GetLineEnd().X, y = GetLineEnd().Y };

            return new PathD() { pointA, pointB};
        }
    
        public List<PointD> GetPoints()
        {
            var pointDs = new List<PointD>();

            foreach (var point in pointsOnLine)
            {
                pointDs.Add(new PointD { x = point.X, y = point.Y});
            }

            return pointDs;
        }
    
        public void Reverse()
        {
            pointsOnLine.Reverse();
        }
    }
}
