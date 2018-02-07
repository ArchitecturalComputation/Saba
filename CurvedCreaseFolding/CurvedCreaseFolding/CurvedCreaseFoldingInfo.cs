using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace CurvedCreaseFolding
{
    public class CurvedCreaseFoldingInfo : GH_AssemblyInfo
    {
        public override string Name => "CurvedCreaseFolding";
        public override Bitmap Icon => null;
        public override string Description => "Simulating curved folding in meshes";
        public override Guid Id => new Guid("4d74befa-0b34-4b65-8410-0d8a36a136f0");
        public override string AuthorName => "Saba Mirmotalebi";
        public override string AuthorContact => "";
    }
}
