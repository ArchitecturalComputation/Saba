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

        public HingeVertex()
        {
        }

        public HingeVertex(List<int> V, double k)
        {
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
            //Pts.Last() will be the main vertex being analyzed

            var facePlanes = new Plane[L - 1];

            for (int i = 0; i < L - 1; i++)
            {
                int j = (i == (L - 1) - 1) ? 0 : i + 1;
                facePlanes[i] = new Plane(Pts.Last(), Pts[i], Pts[j]);
            }

            var angles = new double[L - 1];
            for (int i = 0; i < L - 1; i++)
            {
                int j = (i == (L - 1) - 1) ? 0 : i + 1;
                var cross = Vector3d.CrossProduct(facePlanes[i].Normal, facePlanes[j].Normal);
                angles[i] = Vector3d.VectorAngle(facePlanes[i].Normal, facePlanes[j].Normal, cross);
            }

            double max = 0, max2 = 0;
            int maxIndex = -1, maxIndex2 = -1;

            for (int i = 0; i < L - 1; i++)
                if (angles[i] > max)
                {
                    max = angles[i];
                    maxIndex = i;
                }

            for (int i = 0; i < L - 1; i++)
                if (i != maxIndex && angles[i] > max2)
                {
                    max2 = angles[i];
                    maxIndex2 = i;
                }

            var index1 = (maxIndex < maxIndex2) ? maxIndex : maxIndex2;
            var index2 = (maxIndex > maxIndex2) ? maxIndex : maxIndex2;

            var planarize1 = new List<int>();
            var planarize2 = new List<int>();
            for (int i = 0; i < L - 1; i++)
            {
                if (i < index1) planarize2.Add(i);
                if (i > index1 && i < index2) planarize1.Add(i);
                if (i > index2) planarize2.Add(i);
                if (i == index1 || i == index2)
                {
                    planarize1.Add(i);
                    planarize2.Add(i);
                }
            }
            planarize1.Add(L - 1);
            planarize2.Add(L - 1);

            Plane P1 = new Plane();
            Plane.FitPlaneToPoints(planarize1.Select(i => Pts[i]), out P1);

            Plane P2 = new Plane();
            Plane.FitPlaneToPoints(planarize2.Select(i => Pts[i]), out P2);

            for (int i = 0; i < L; i++)
            {
                if (i == index1 || i == index2 || i == L - 1)
                {
                    Move[i] = Vector3d.Add(P1.ClosestPoint(Pts[i]) - Pts[i], P2.ClosestPoint(Pts[i]) - Pts[i]);
                }
                else if (planarize1.Contains(i))
                    Move[i] = P1.ClosestPoint(Pts[i]) - Pts[i];
                else
                    Move[i] = P2.ClosestPoint(Pts[i]) - Pts[i];

                Weighting[i] = Strength;
            }
        }

    }
}