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
        Mesh mesh;
        List<Curve> folds;
        double angle = 0;
        public Mesh OutMesh { get; private set; }
        public Mesh OutMeshFlat { get; private set; }

        public FoldingSimulation(Mesh _mesh, double _angle, List<Curve> _folds)
        {
            mesh = _mesh;
            folds = _folds;
            angle = _angle;
            Folding();
        }

        void Folding()
        {
            mesh.Weld(PI);
            mesh.Faces.ConvertQuadsToTriangles();

            var simulation = new PhysicalSystem();
            var particles = mesh.TopologyVertices
                .Select(p => new Point3d(p.X, p.Y, p.Z))
                .ToList();
            simulation.SetParticleList(particles);

            var goals = new List<IGoal>();
            var goals2D = new List<IGoal>();

            AddEdgeLengthGoals(mesh, goals);
            AddDevelopableGoals(goals);
            if (folds != null) AddHingeGoals(goals);
            Simulation(simulation, goals);
            OutMesh = ConstructMesh(simulation);

            AddEdgeLengthGoals(OutMesh, goals2D);
            AddFlattenGoals(OutMesh, goals2D);
            Simulation(simulation, goals2D);
            OutMeshFlat = ConstructMesh(simulation);
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

        IGoal HingeGoals(int topologyIndex, List<Curve> folds, double angle, double distance)
        {
            var points = mesh.TopologyVertices.ToArray();
            var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(topologyIndex);
            double curveParam = 0.0, tangentAngle = 0.0;
            var midPoint = (float)0.5 * points[edgeVertices.I] + (float)0.5 * points[edgeVertices.J];
            foreach (Curve c in folds)
                if (c.ClosestPoint(midPoint, out curveParam, 0.3 * distance))
                {
                    var curveTangent = c.TangentAt(curveParam);
                    var edgeTangent = mesh.TopologyEdges.EdgeLine(topologyIndex).UnitTangent;
                    tangentAngle = Vector3d.VectorAngle(curveTangent, edgeTangent);
                    break;
                }

            if (curveParam == 0.0) return null;
            if (tangentAngle > PI * 0.25 && tangentAngle < PI * 0.75) return null;

            int[] faces = mesh.TopologyEdges.GetConnectedFaces(topologyIndex, out bool[] direction);
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

        void AddFlattenGoals(Mesh _mesh, List<IGoal> _goals)
        {
            var indices = new List<int>();
            var p = new Plane(new Point3d(0, 0, 0), new Vector3d(0, 0, 1));
            for (int i = 0; i < _mesh.Vertices.Count; i++)
                indices.Add(i);
            var onPlane = new OnPlane(indices, p, 1.0);
            _goals.Add(onPlane);
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

            var colors = GetMeshColors(outMesh);
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
                outMesh.VertexColors.SetColor(i, colors[i]);

            //outMesh.Unweld(0.01, true);
            return outMesh;
        }

        System.Drawing.Color[] GetMeshColors(Mesh _mesh)
        {
            var colors = new System.Drawing.Color[mesh.Vertices.Count];
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                var index = _mesh.TopologyVertices.MeshVertexIndices(i).First();
                var indices = _mesh.TopologyVertices.MeshVertexIndices(i);

                var vColor = MeshVertexColor(_mesh, i);
                foreach (int v in indices) colors[v] = vColor;
            }
            return colors;
        }

        System.Drawing.Color MeshVertexColor(Mesh _mesh, int topologyIndex)
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
            var R = (int)(Abs(curvature) * 255 / (2 * PI));
            var GB = 255 - R;
            var color = System.Drawing.Color.FromArgb(R, GB, GB);
            var col = new Rhino.Display.ColorHSV(R, 1, 1).ToArgbColor();
            return color;
        }
    }
}
