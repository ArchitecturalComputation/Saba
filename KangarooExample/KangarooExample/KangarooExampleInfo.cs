using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace KangarooExample
{
    public class KangarooExampleInfo : GH_AssemblyInfo
    {
        public override string Name => "KangarooExample";
        public override Bitmap Icon => null;
        public override string Description => "This plugin is a kangaroo simulation.";
        public override Guid Id => new Guid("b0959c67-bdf1-4e15-9896-7f2aac916bd9");
        public override string AuthorName => "Architectural Computation";
        public override string AuthorContact => "";

    }
}
