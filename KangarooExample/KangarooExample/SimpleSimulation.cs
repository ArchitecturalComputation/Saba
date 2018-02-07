using System;
using System.Linq;
using System.Collections.Generic;
using Rhino.Geometry;
using KangarooSolver;
using KangarooSolver.Goals;

namespace KangarooExample
{
    internal class SimpleSimulation
    {
        public Mesh OutMesh { get; private set; }

        public SimpleSimulation(Mesh mesh, double force)
        {
            Simulate(mesh, force);
        }

        void Simulate(Mesh mesh, double force)
        {
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

                if (edgeCount <= 3)
                {
                    Point3d p = mesh.TopologyVertices[i];
                    var anchor = new Anchor(i, p, 10000);
                    goals.Add(anchor);
                }
            }


            // unary 
            Vector3d forceVector = Vector3d.ZAxis * force;

            for (int i = 0; i < mesh.TopologyVertices.Count; i++)
            {
                var unary = new Unary(i, forceVector);
                goals.Add(unary);
            }


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

//var newVertices = new List<Point3d>();

//for (int i = 0; i < mesh.Vertices.Count; i++)
//{
//    int j = mesh.TopologyVertices.TopologyVertexIndex(i);
//    newVertices.Add(outParticles[j]);
//}