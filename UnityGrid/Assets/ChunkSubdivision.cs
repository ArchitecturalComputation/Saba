using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

class ChunkSubdivision
{
    public List<List<Cell>> Chunks { get { return _chunks.ToList(); } }

    private CubicGrid _grid;
    private List<List<Cell>> _chunks;


    public ChunkSubdivision(float cellSize, BoundingSphere sphere, int chunkCount)
    {
        _chunks = new List<List<Cell>>();
        _grid = new CubicGrid(cellSize, sphere);
    }

    public void CreateChunks(int count)
    {
        for (int i = 0; i < count; i++)
        {
            var start = _grid.GetRandomCell();
            CreateChunk(start, 400);
        }
    }

    public bool AllCellsAdded()
    {
        int count = Chunks.SelectMany(c => c).Count();
        return count == _grid.ValidCellCount;
    }

    void CreateChunk(Cell start, int maxCells)
    {
        if (!start.IsValid)
            return;

        var cells = new List<Cell>();
        _chunks.Add(cells);

        cells.Add(start);

        var currentCell = start;

        while (cells.Count < maxCells)
        {
            var neighbours = _grid.GetNeighbors(currentCell);
            var validNeighbours = neighbours.Where(c => c.IsValid).ToList();

            if (validNeighbours.Count == 0) return;

            foreach (var cell in validNeighbours)
                cell.IsValid = false;

            int randomIndex = UnityEngine.Random.Range(0, validNeighbours.Count);
            currentCell = validNeighbours[randomIndex];
            cells.AddRange(validNeighbours);
        }

    }
}