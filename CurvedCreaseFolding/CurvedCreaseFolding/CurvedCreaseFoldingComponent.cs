using System;
using System.Collections.Generic;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace CurvedCreaseFolding
{
    public class CurvedCreaseFoldingComponent : GH_Component
    {
        public CurvedCreaseFoldingComponent() : base("CurvedCreaseFolding", "CrvCrsFld", "Make a triangular mesh developable and optionally fold along input fold curves", "Folding", "CurvedCreases") { }
        protected override System.Drawing.Bitmap Icon => null;
        public override Guid ComponentGuid => new Guid("0307911d-e027-4751-b6ee-b1e5cfa9b3c7");
        public override GH_Exposure Exposure => GH_Exposure.primary;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Initial mesh to make developable", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Mesh Subdivision", "MS", "Level of subdivion on initial mesh", GH_ParamAccess.item, 0);
            pManager.AddCurveParameter("Fold Curves", "FC", "Lines to be folded", GH_ParamAccess.list);
            pManager.AddNumberParameter("Angle", "A", "Angle of folding in Radians", GH_ParamAccess.item, 0.0);

            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh3D", "M3D", "Final 3D mesh that is flattenable", GH_ParamAccess.item);
            pManager.AddLineParameter("Folding Edges", "FE", "Edges being folded", GH_ParamAccess.list);
            pManager.AddMeshParameter("Mesh2D", "M2D", "Final mesh flattened", GH_ParamAccess.item);
            pManager.AddCurveParameter("Fold curves", "FC", "Curves to fold along", GH_ParamAccess.list);
            pManager.AddCurveParameter("Boundary curves", "BC", "Curves to cut along", GH_ParamAccess.list);
            pManager.AddNumberParameter("Gaussian curvature", "GC", "Gaussian curvature at vertex points", GH_ParamAccess.list);
            pManager.AddNumberParameter("Kinetic Energy", "KE", "Final kinetic energy after simulation", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            int subdivs = 0;
            var folds = new List<Curve>();
            double angle = 0.0;

            DA.GetData(0, ref mesh);
            DA.GetData(1, ref subdivs);
            DA.GetDataList(2, folds);
            DA.GetData(3, ref angle);

            var simulation = new FoldingSimulation(mesh, subdivs, folds, angle);

            var outMesh = simulation.OutMesh;
            DA.SetData(0, outMesh);

            var foldings = simulation.Foldings;
            DA.SetDataList(1, foldings);

            var outMeshFlat = simulation.OutMeshFlat;
            DA.SetData(2, outMeshFlat);

            var foldCurves = simulation.FoldCurves;
            DA.SetDataList(3, foldCurves);

            var boundaryCurves = simulation.BoundaryCurves;
            DA.SetDataList(4, boundaryCurves);

            var curvatures = simulation.GaussianCurvatures;
            DA.SetDataList(5, curvatures);

            var kineticEnergy = simulation.KineticEnergy;
            DA.SetData(6, kineticEnergy);
        }
    }
}
