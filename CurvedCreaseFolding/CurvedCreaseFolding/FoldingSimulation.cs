using System;
using System.Linq;
using Rhino.Geometry;
using KangarooSolver;
using KangarooSolver.Goals;
using System.Collections.Generic;

namespace CurvedCreaseFolding
{
    class FoldingSimulation
    {
        public Mesh OutMesh { get; private set; }

        public FoldingSimulation(Mesh mesh, double force)
        {
            Simulate(mesh, force);
        }

        void Simulate(Mesh mesh, double force)
        {
            mesh.Weld(0.000001);
            mesh.Faces.ConvertQuadsToTriangles();
            var simulation = new PhysicalSystem();
            var particles = mesh.TopologyVertices
                .Select(p => new Point3d(p.X, p.Y, p.Z))
                .ToList();

            simulation.SetParticleList(particles);
            var goals = new List<IGoal>();

            // edges
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                var pair = mesh.TopologyEdges.GetTopologyVertices(i);
                Vector3d vector = mesh.TopologyVertices[pair.I] - mesh.TopologyVertices[pair.J];
                double length = vector.Length;
                var spring = new Spring(pair.I, pair.J, length, 1.0);
                goals.Add(spring);
            }

            // anchors
            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                int edgeCount = mesh.TopologyVertices.ConnectedEdgesCount(i);
                if (edgeCount <= 4)
                {
                    Point3d p = mesh.TopologyVertices[i];
                    var anchor = new Anchor(i, p, 10000);
                    goals.Add(anchor);
                }
            }

            //hinges.saba
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                var points = mesh.TopologyVertices.Select(p => new Point3d(p.X, p.Y, p.Z)).ToArray();
                var faceIndices = mesh.TopologyEdges.GetConnectedFaces(i);
                if (faceIndices.Length == 2)
                {
                    Point3f[] v1 = new Point3f[4];
                    Point3f[] v2 = new Point3f[4];
                    mesh.Faces.GetFaceVertices(faceIndices[0], out v1[0], out v1[1], out v1[2], out v1[3]);
                    mesh.Faces.GetFaceVertices(faceIndices[1], out v2[0], out v2[1], out v2[2], out v2[3]);
                    var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(i);
                    Point3f a = new Point3f();
                    Point3f b = new Point3f();
                    for (int j = 0; j < 4; j++)
                    {
                        if (!v1[j].Equals(edgeVertices.I) && !v1[j].Equals(edgeVertices.J))
                            a = v1[j];
                        if (!v2[j].Equals(edgeVertices.I) && !v2[j].Equals(edgeVertices.J))
                            b = v2[j];
                    }
                    goals.Add(new Hinge(points[edgeVertices.I], points[edgeVertices.J], a, b, Math.PI / 3, 1.0));
                }
            }

            // hinges.vicente
            // assumes a triangulated mesh.
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                int[] faces = mesh.TopologyEdges.GetConnectedFaces(i, out bool[] direction);
                if (faces.Length != 2) continue;

                if (!direction[0]) faces.Reverse();
                var faceVertices = faces.Select(f => mesh.TopologyVertices.IndicesFromFace(f));
                var edgeVertices = mesh.TopologyEdges.GetTopologyVertices(i);
                var opposites = faceVertices.Select(f => f.First(v => !edgeVertices.Contains(v))).ToArray();

                var hinge = new Hinge(edgeVertices.I, edgeVertices.J, opposites.First(), opposites.Last(), Math.PI / 3.0, 1.0);
                goals.Add(hinge);
            }



            // unary 
            //Vector3d forceVector = Vector3d.ZAxis * force;

            //for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            //{
            //    var unary = new Unary(i, forceVector);
            //    goals.Add(unary);
            //}

            // simulation
            int counter = 0;
            double threshold = 1e-9;
            do
            {
                simulation.Step(goals, true, threshold);
                counter++;
            } while (simulation.GetvSum() > threshold && counter < 200);

            var outMesh = new Mesh();
            outMesh.Faces.AddFaces(mesh.Faces);

            var outParticles = simulation.GetPositionArray();
            var outVertices = Enumerable.Range(0, mesh.Vertices.Count)
                .Select(i => outParticles[mesh.TopologyVertices.TopologyVertexIndex(i)]);

            outMesh.Vertices.AddVertices(outVertices);
            OutMesh = outMesh;
        }
    }
}
