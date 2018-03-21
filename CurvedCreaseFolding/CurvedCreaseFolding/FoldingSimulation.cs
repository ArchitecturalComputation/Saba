using System;
using System.Linq;
using Rhino.Geometry;
using Rhino.Collections;
using KangarooSolver;
using KangarooSolver.Goals;
using System.Collections.Generic;
using static System.Math;

namespace CurvedCreaseFolding
{
    class FoldingSimulation
    {
        Mesh mesh = null;
        List<Curve> folds;
        double angle;
        List<int> foldedEdges = new List<int>();

        public Mesh OutMesh { get; private set; }
        public Mesh OutMeshFlat { get; private set; }
        public List<double> GaussianCurvatures { get; private set; }
        public List<Line> Foldings { get; private set; }
        public float KineticEnergy { get; private set; }
        public List<Curve> FoldCurves { get; private set; }
        public List<Curve> BoundaryCurves { get; private set; }

        public FoldingSimulation(Mesh _mesh, int _subdivs, List<Curve> _folds, double _angle)
        {
            _mesh.Weld(PI);
            _mesh.Faces.ConvertQuadsToTriangles();
            mesh = _mesh;
            var tempMesh = _mesh;
            for (int i = 0; i < _subdivs; i++)
            {
                mesh = MeshSubdivision(tempMesh);
                tempMesh.CopyFrom(mesh);
            }
            folds = _folds;
            angle = _angle;

            Fold();
        }

        void Fold()
        {
            var simulation = new PhysicalSystem();
            var particles = mesh.TopologyVertices
                .Select(p => new Point3d(p.X, p.Y, p.Z))
                .ToList();
            simulation.SetParticleList(particles);

            var goals = new List<IGoal>();
            var goals2D = new List<IGoal>();

            AddEdgeLengthGoals(mesh, goals);
            AddDevelopableGoals(goals);
            if (folds.Count != 0)
            {
                AddHingeGoals(goals);
                AddHingeVertexGoals(goals);
            }
            Simulation(simulation, goals);
            OutMesh = ConstructMesh(simulation);
            if (folds.Count == 0) GetFoldedEdges();

            AddEdgeLengthGoals(OutMesh, goals2D);
            AddHingeFlattenGoals(goals2D);
            Simulation(simulation, goals2D);
            OutMeshFlat = ConstructFlatMesh(simulation);

            KineticEnergy = ComputeEnergy();
            SetFoldCurves();
            SetBoundaryCurves();
        }

        Mesh MeshSubdivision(Mesh _mesh)
        {
            var points = _mesh.Vertices.ToPoint3dArray();
            List<Point3d> subdividedVertices = points.ToList();
            List<MeshFace> subdividedFaces = new List<MeshFace>();
            var vCount = _mesh.Vertices.Count;

            foreach (MeshFace f in _mesh.Faces)
            {
                IEnumerable<int> uniqueIndices = new List<int> { f.A, f.B, f.C, f.D }.Distinct();
                var vPoints = new List<Point3d>(uniqueIndices.Select(i => points[i])).ToArray(); //vertex points
                var mPoints = new Point3d[3]; //midpoints
                var faces = new List<MeshFace>();
                for (int i = 0; i < 3; i++)
                {
                    //constructing mid point vertices
                    var j = (i + 1) % 3;
                    mPoints[i] = (vPoints[i] + vPoints[j]) / 2.0;

                    //constructing corner faces
                    var k = ((i - 1) + 3) % 3;
                    var cornerFace = new MeshFace(uniqueIndices.ElementAt(i), vCount + i, vCount + k);
                    faces.Add(cornerFace);
                }

                //constructing middle face
                var midFace = new MeshFace(vCount, vCount + 1, vCount + 2);
                faces.Add(midFace);

                subdividedVertices.AddRange(mPoints);
                subdividedFaces.AddRange(faces);
                vCount += 3;
            }

            var subdividedMesh = new Mesh();
            subdividedMesh.Vertices.AddVertices(subdividedVertices);
            subdividedMesh.Faces.AddFaces(subdividedFaces);
            return subdividedMesh;
        }

        void AddEdgeLengthGoals(Mesh _mesh, List<IGoal> _goals)
        {
            for (int i = 0; i < _mesh.TopologyEdges.Count; i++)
            {
                var pair = _mesh.TopologyEdges.GetTopologyVertices(i);
                Vector3d vector = _mesh.TopologyVertices[pair.I] - _mesh.TopologyVertices[pair.J];
                double length = vector.Length;
                var spring = new Spring(pair.I, pair.J, length, 1);
                _goals.Add(spring);
            }
        }

        void AddHingeGoals(List<IGoal> _goals)
        {
            double minDist = 0;
            for (int i = 0; i < mesh.TopologyEdges.Count; i++) minDist += mesh.TopologyEdges.EdgeLine(i).Length;
            minDist /= mesh.TopologyEdges.Count;

            var hingeGoals = new List<IGoal>();
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                var tempGoal = HingeGoals(i, folds, angle, minDist);
                if (tempGoal != null) hingeGoals.Add(tempGoal);
            }
            _goals.AddRange(hingeGoals);
        }

        IGoal HingeGoals(int topologyEdgeIndex, List<Curve> folds, double angle, double distance)
        {
            var points = mesh.TopologyVertices.ToArray();
            var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(topologyEdgeIndex);
            double curveParam = 0.0, tangentAngle = 0.0;
            var midPoint = (float)0.5 * points[edgeVertices.I] + (float)0.5 * points[edgeVertices.J];
            foreach (Curve c in folds)
                if (c.ClosestPoint(midPoint, out curveParam, 0.3 * distance))
                {
                    var curveTangent = c.TangentAt(curveParam);
                    var edgeTangent = mesh.TopologyEdges.EdgeLine(topologyEdgeIndex).UnitTangent;
                    tangentAngle = Vector3d.VectorAngle(curveTangent, edgeTangent);
                    break;
                }

            if (curveParam == 0.0) return null;
            if (tangentAngle > PI * 0.15 && tangentAngle < PI * 0.85) return null;

            foldedEdges.Add(topologyEdgeIndex);

            int[] faces = mesh.TopologyEdges.GetConnectedFaces(topologyEdgeIndex, out bool[] direction);
            if (faces.Length != 2) return null;

            int r = 0;
            if (!direction[0]) r = 1;

            var faceVertices2 = faces.Select(f => mesh.TopologyVertices.IndicesFromFace(f)).ToArray();
            var faceVertices = new[] { faceVertices2[r], faceVertices2[1 - r] };
            var opposites = faceVertices.Select(f => f.First(v => !edgeVertices.Contains(v))).ToArray();

            var hinge = new Hinge(edgeVertices.I, edgeVertices.J, opposites.First(), opposites.Last(), -angle, 1.0);
            return hinge;
        }

        void AddDevelopableGoals(List<IGoal> _goals)
        {
            var naked = mesh.GetNakedEdgePointStatus();

            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                var index = mesh.TopologyVertices.MeshVertexIndices(i).First();
                if (naked[index]) continue;

                var developableGoals = DevelopableGoals(i);
                _goals.AddRange(developableGoals);
            }
        }

        IEnumerable<IGoal> DevelopableGoals(int topologyIndex)
        {
            var neighbors = mesh.TopologyVertices.ConnectedTopologyVertices(topologyIndex, true);
            var vectors = neighbors.Select(n => mesh.TopologyVertices[n] - mesh.TopologyVertices[topologyIndex])
                .Select(vf => new Vector3d(vf.X, vf.Y, vf.Z)).ToList();

            var angles = new double[neighbors.Length];

            for (int i = 0; i < neighbors.Length; i++)
            {
                int j = (i == neighbors.Length - 1) ? 0 : i + 1;
                angles[i] = Vector3d.VectorAngle(vectors[i], vectors[j]);
            }

            double angleSum = angles.Sum();
            double factor = (PI * 2) / angleSum;

            for (int i = 0; i < neighbors.Length; i++)
            {
                int j = (i == neighbors.Length - 1) ? 0 : i + 1;
                var finalAngle = angles[i] * factor;
                var angle = new Angle(1, finalAngle, topologyIndex, neighbors[i], topologyIndex, neighbors[j]);
                yield return angle;
            }
        }

        void AddHingeVertexGoals(List<IGoal> _goals)
        {
            var tempGoals = new List<IGoal>();
            var naked = mesh.GetNakedEdgePointStatus();
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                if (naked[i]) continue;
                var neighbors = mesh.TopologyVertices.ConnectedTopologyVertices(i, true).ToList();
                neighbors.Add(i);

                var vertexHinge = new HingeVertex(neighbors, 0.2);
                tempGoals.Add(vertexHinge);
            }
            _goals.AddRange(tempGoals);
        }

        void AddHingeFlattenGoals(List<IGoal> _goals)
        {
            var hingeGoals = new List<IGoal>();
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                var tempGoal = HingeFlattenGoals(i);
                if (tempGoal != null) hingeGoals.Add(tempGoal);
            }

            _goals.AddRange(hingeGoals);
        }

        IGoal HingeFlattenGoals(int topologyIndex)
        {
            var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(topologyIndex);

            int[] faces = mesh.TopologyEdges.GetConnectedFaces(topologyIndex, out bool[] direction);
            if (faces.Length != 2) return null;

            int r = 0;
            if (!direction[0]) r = 1;

            var faceVertices2 = faces.Select(f => mesh.TopologyVertices.IndicesFromFace(f)).ToArray();
            var faceVertices = new[] { faceVertices2[r], faceVertices2[1 - r] };
            var opposites = faceVertices.Select(f => f.First(v => !edgeVertices.Contains(v))).ToArray();

            var hinge = new Hinge(edgeVertices.I, edgeVertices.J, opposites.First(), opposites.Last(), 0, 1.0);
            return hinge;
        }

        void Simulation(PhysicalSystem _simulation, List<IGoal> _goals)
        {
            int counter = 0;
            double threshold = 1e-9;
            do
            {
                _simulation.Step(_goals, true, threshold);
                counter++;
            } while (_simulation.GetvSum() > threshold && counter < 200);
        }

        Mesh ConstructMesh(PhysicalSystem _simulation)
        {
            var outMesh = new Mesh();
            outMesh.Faces.AddFaces(mesh.Faces);

            var outParticles = _simulation.GetPositionArray();
            var outVertices = Enumerable.Range(0, mesh.Vertices.Count)
                .Select(i => outParticles[mesh.TopologyVertices.TopologyVertexIndex(i)]);
            outMesh.Vertices.AddVertices(outVertices);

            var gaussianCurvatures = GetMeshCurvatures(outMesh);
            GaussianCurvatures = gaussianCurvatures.ToList();

            var foldings = new List<Line>();
            foreach (int i in foldedEdges) foldings.Add(outMesh.TopologyEdges.EdgeLine(i));
            Foldings = foldings;

            return outMesh;
        }

        Mesh ConstructFlatMesh(PhysicalSystem _simulation)
        {
            var outMesh = new Mesh();
            outMesh.Faces.AddFaces(mesh.Faces);

            var outParticles = _simulation.GetPositionArray();

            Plane.FitPlaneToPoints(outParticles.ToList(), out Plane originPlane);
            var XY = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
            var orientToXY = Transform.PlaneToPlane(originPlane, XY);

            var outParticlesPlanarized = new Point3d[outParticles.Length];
            for (int i = 0; i < outParticles.Length; i++)
            {
                outParticlesPlanarized[i] = originPlane.ClosestPoint(outParticles[i]);
                outParticlesPlanarized[i].Transform(orientToXY);
            }

            var outVertices = Enumerable.Range(0, mesh.Vertices.Count)
                .Select(i => outParticlesPlanarized[mesh.TopologyVertices.TopologyVertexIndex(i)]);
            outMesh.Vertices.AddVertices(outVertices);

            return outMesh;
        }

        void GetFoldedEdges()
        {
            //only used when folding curves aren't input, in that instance they are computet automatically when folding
            var points = OutMesh.Vertices.ToArray();
            for (int i = 0; i < OutMesh.TopologyEdges.Count; i++)
            {
                int[] faces = OutMesh.TopologyEdges.GetConnectedFaces(i);
                if (faces.Length != 2) continue;

                var facePlanes = new Plane[2];
                for (int j = 0; j < 2; j++)
                {
                    var f = OutMesh.Faces.GetFace(faces[j]);
                    var uniqueIndices = new List<int> { f.A, f.B, f.C, f.D }.Distinct();
                    var v = uniqueIndices.Select(index => points[index]).ToArray();
                    facePlanes[j] = new Plane(v[0], v[1], v[2]);
                }

                var angleBetweenFaces = Vector3d.VectorAngle(facePlanes[0].Normal, facePlanes[1].Normal);
                if (angleBetweenFaces < PI * 0.25) continue;

                foldedEdges.Add(i);
            }
            var foldings = new List<Line>();
            foreach (int i in foldedEdges) foldings.Add(OutMesh.TopologyEdges.EdgeLine(i));
            Foldings = foldings;
        }

        double[] GetMeshCurvatures(Mesh _mesh)
        {
            var naked = mesh.GetNakedEdgePointStatus();
            var curvatures = new double[mesh.Vertices.Count];
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                var index = _mesh.TopologyVertices.MeshVertexIndices(i).First();
                var indices = _mesh.TopologyVertices.MeshVertexIndices(i);

                var vCurvature = MeshVertexCurvature(_mesh, i);
                if (naked[index]) vCurvature = 0;
                foreach (int v in indices) curvatures[v] = vCurvature;
            }
            return curvatures;
        }

        double MeshVertexCurvature(Mesh _mesh, int topologyIndex)
        {
            var neighbors = _mesh.TopologyVertices.ConnectedTopologyVertices(topologyIndex, true);
            var vectors = neighbors.Select(n => _mesh.TopologyVertices[n] - _mesh.TopologyVertices[topologyIndex])
                .Select(vf => new Vector3d(vf.X, vf.Y, vf.Z)).ToList();

            var angles = new double[neighbors.Length];

            for (int i = 0; i < neighbors.Length; i++)
            {
                int j = (i == neighbors.Length - 1) ? 0 : i + 1;
                angles[i] = Vector3d.VectorAngle(vectors[i], vectors[j]);
            }

            var curvature = PI * 2 - angles.Sum();
            return curvature;
        }

        float ComputeEnergy()
        {
            float energy = 0;

            var iPoints = mesh.TopologyVertices.ToArray();
            var oPoints = OutMesh.TopologyVertices.ToArray();

            for (int i = 0; i < iPoints.Length; i++)
            {
                var difference = oPoints[i] - iPoints[i];
                energy += difference.SquareLength;
            }

            return energy;
        }

        void SetFoldCurves()
        {
            var foldCurves = new List<Curve>();
            IEnumerable<Curve> foldings = foldedEdges.Select(i => OutMeshFlat.TopologyEdges.EdgeLine(i).ToNurbsCurve());
            var polylines = Curve.JoinCurves(foldings);
            foreach (Curve c in polylines)
            {
                //var discontinuities = new List<Double>();
                //for (int i = 0; i <= c.Domain.Length; i++)
                //    discontinuities.Add(i);

                c.Domain = new Interval(0, 1);
                var discontinuities = new List<Double>();
                discontinuities.Add(0);
                Boolean check = true;
                //double m = 0; // to create midpoints between discontinuities to create a more accurate curve
                while (check)
                {
                    check = c.GetNextDiscontinuity((Continuity)4, discontinuities.Last(), 1, out double d);
                    //if (m != 0 && check) discontinuities.Add((m + d) / 2);
                    //m = d;
                    if (check) discontinuities.Add(d);
                }
                discontinuities.Add(1);

                IEnumerable<Point3d> dPoints = discontinuities.Select(d => c.PointAt(d));
                var curve = Curve.CreateControlPointCurve(dPoints, 3);
                //var curve = Curve.CreateInterpolatedCurve(dPoints, 3, (CurveKnotStyle) 4);
                foldCurves.Add(curve);
            }
            FoldCurves = foldCurves;
        }

        void SetBoundaryCurves()
        {
            var boundaryCurves = new List<Curve>();
            var e = OutMesh.GetNakedEdges();
            var outerEdges = new List<Curve>();
            for (int i = 0; i < e.Length; i++)
                outerEdges.Add(e[i].ToNurbsCurve());

            var outline = Curve.JoinCurves(outerEdges);
            PolylineCurve.JoinCurves(outerEdges);
            for (int i = 0; i < OutMesh.TopologyEdges.Count; i++)
            {
                //for (int i = 0; i < mesh.TopologyVertices.Count; i++)
                //{
                //    var index = mesh.TopologyVertices.MeshVertexIndices(i).First();
                //    if (naked[index]) continue;

                //    var developableGoals = DevelopableGoals(i);
                //    _goals.AddRange(developableGoals);
                //}
            }

            IEnumerable<Curve> foldings = foldedEdges.Select(i => OutMeshFlat.TopologyEdges.EdgeLine(i).ToNurbsCurve());
            var polylines = Curve.JoinCurves(foldings);
            foreach (Curve c in polylines)
            {
                c.Domain = new Interval(0, 1);
                var discontinuities = new List<Double>();
                discontinuities.Add(0);
                Boolean check = true;
                while (check)
                {
                    check = c.GetNextDiscontinuity((Continuity)4, discontinuities.Last(), 1, out double d);
                    if (check) discontinuities.Add(d);
                }
                discontinuities.Add(1);

                IEnumerable<Point3d> dPoints = discontinuities.Select(d => c.PointAt(d));
                var curve = Curve.CreateInterpolatedCurve(dPoints, 3);
                boundaryCurves.Add(curve);
            }
            BoundaryCurves = boundaryCurves;
        }
    }
}
