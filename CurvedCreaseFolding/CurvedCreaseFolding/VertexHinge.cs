using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace KangarooSolver.Goals
{
    public class HingeVertex : GoalObject
    {
        public double Strength;
        public double TargetArea;
        int centerIndex;

        public HingeVertex()
        {
        }

        public HingeVertex(int index, List<int> V, double k)
        {
            centerIndex = index;
            int L = V.Count;
            PIndex = V.ToArray();
            Move = new Vector3d[L];
            Weighting = new double[L];
            for (int i = 0; i < L; i++)
                Weighting[i] = k;
            Strength = k;
        }

        public override void Calculate(List<KangarooSolver.Particle> p)
        {
            int L = PIndex.Length;
            Point3d[] Pts = new Point3d[L];
            for (int i = 0; i < L; i++)
                Pts[i] = p[PIndex[i]].Position;
            var center = p[centerIndex].Position;

            var convex = new List<int>();
            var concave = new List<int>();
            var neighbors = new List<Neighbor>();
            for (int i = 0; i < L; i++)
            {
                int nexti = (i == L - 1) ? 0 : i + 1;
                int previ = (i == 0) ? L - 1 : i - 1;

                Point3d neighbour = Pts[i];
                Point3d next = Pts[nexti];
                Point3d prev = Pts[previ];

                Vector3d edge = neighbour - center;
                Vector3d va = next - center;
                Vector3d vb = prev - center;

                Vector3d na = Vector3d.CrossProduct(va, edge);
                Vector3d nb = Vector3d.CrossProduct(edge, vb);
                var angle = Math.PI - Vector3d.VectorAngle(na, nb);

                Vector3d facesOrtho = Vector3d.CrossProduct(na, nb);
                bool isConvex = Vector3d.VectorAngle(facesOrtho, edge) < Math.PI;

                if (isConvex) convex.Add(i);
                else concave.Add(i);

                neighbors.Add(new Neighbor(i, angle));
            }

            var convexNeighbors = convex.Select(i => neighbors[i]).ToArray();
            var convexAngles = convexNeighbors.Select(n => Math.PI - Math.Abs(n.Angle)).ToArray();
            Array.Sort(convexAngles, convexNeighbors);

            var concaveNeighbors = concave.Select(i => neighbors[i]).ToArray();
            var concaveAngles = concaveNeighbors.Select(n => Math.PI - Math.Abs(n.Angle)).ToArray();
            Array.Sort(concaveAngles, concaveNeighbors);

            var convexError = (convexNeighbors.Length > 2) ?
                Math.Pow(Math.PI - Math.Abs(convexNeighbors[0].Angle), 2) +
                Math.Pow(Math.PI - Math.Abs(convexNeighbors[1].Angle), 2) :
                double.MaxValue;

            var concaveError = (concaveNeighbors.Length > 2) ?
                Math.Pow(Math.PI - Math.Abs(concaveNeighbors[0].Angle), 2) +
                Math.Pow(Math.PI - Math.Abs(concaveNeighbors[1].Angle), 2) :
                double.MaxValue;

            var top2seams = (convexError < concaveError) ?
                new Neighbor[] { convexNeighbors[0], convexNeighbors[1] } :
                new Neighbor[] { concaveNeighbors[0], concaveNeighbors[1] };
            Array.Sort(top2seams.Select(n => n.Index).ToArray(), top2seams);

            var planarize1 = new List<int>();
            var planarize2 = new List<int>();
            for (int i = 0; i < L; i++)
            {
                if (i < top2seams[0].Index) planarize2.Add(i);
                if (i > top2seams[0].Index && i < top2seams[1].Index) planarize1.Add(i);
                if (i > top2seams[1].Index) planarize2.Add(i);
                if (i == top2seams[0].Index || i == top2seams[1].Index)
                {
                    planarize1.Add(i);
                    planarize2.Add(i);
                }
            }

            Plane P1 = new Plane();
            Plane.FitPlaneToPoints(planarize1.Select(i => Pts[i]), out P1);

            Plane P2 = new Plane();
            Plane.FitPlaneToPoints(planarize2.Select(i => Pts[i]), out P2);

            for (int i = 0; i < L; i++)
            {
                if (i == top2seams[0].Index || i == top2seams[1].Index)
                    Move[i] = Vector3d.Add(P1.ClosestPoint(Pts[i]) - Pts[i], P2.ClosestPoint(Pts[i]) - Pts[i]);
                else if (planarize1.Contains(i))
                    Move[i] = P1.ClosestPoint(Pts[i]) - Pts[i];
                else
                    Move[i] = P2.ClosestPoint(Pts[i]) - Pts[i];

                Weighting[i] = Strength;
            }
        }
    }

    class Neighbor
    {
        public int Index { get; private set; }
        public double Angle { get; set; }

        public Neighbor(int i, double a)
        {
            Index = i;
            Angle = a;
        }
    }
}