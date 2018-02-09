using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


internal class CubicGrid
{
    public int ValidCellCount { get { return _validCellCount; } }

    Cell[,,] _cells;
    Vector3Int _size;
    int _validCellCount;


    public CubicGrid(float cellSize, BoundingSphere sphere)
    {
        _size = GetSize(cellSize, sphere);
        _cells = new Cell[_size.x, _size.y, _size.z];
        InitCells(cellSize, sphere);
    }

    public List<Cell> GetNeighbors(Cell cell)
    {
        var cells = new List<Cell>();

        var lx = cell.Position.x == 0 ? 0 : -1;
        var ux = cell.Position.x == _size.x - 1 ? 0 : 1;
        var ly = cell.Position.y == 0 ? 0 : -1;
        var uy = cell.Position.y == _size.y - 1 ? 0 : 1;
        var lz = cell.Position.z == 0 ? 0 : -1;
        var uz = cell.Position.z == _size.z - 1 ? 0 : 1;

        for (int z = lz; z <= uz; z++)
            for (int y = ly; y <= uy; y++)
                for (int x = lx; x <= ux; x++)
                {
                    if (x == 0 && y == 0 && z == 0) continue;
                    var index = new Vector3Int(cell.Position.x + x, cell.Position.y + y, cell.Position.z + z);
                    cells.Add(_cells[index.x, index.y, index.z]);
                }

        return cells;
    }

    public Cell GetRandomCell()
    {
        int x = UnityEngine.Random.Range(0, _size.x);
        int y = UnityEngine.Random.Range(0, _size.y);
        int z = UnityEngine.Random.Range(0, _size.z);

        return _cells[x, y, z];
    }

    private void InitCells(float cellSize, BoundingSphere sphere)
    {
        _validCellCount = 0;
        var origin = sphere.position - Vector3.one * sphere.radius;

        for (int z = 0; z < _size.z; z++)
            for (int y = 0; y < _size.y; y++)
                for (int x = 0; x < _size.x; x++)
                {
                    var index = new Vector3Int(x, y, z);
                    var pos = new Vector3(index.x, index.y, index.z) * cellSize + origin;
                    var isValid = (sphere.position - pos).magnitude < sphere.radius;
                    if (isValid) _validCellCount++;

                    var cell = new Cell()
                    {
                        Position = index,
                        WorldPosition = pos,
                        IsValid = isValid
                    };

                    _cells[x, y, z] = cell;
                }
    }

    private Vector3Int GetSize(float cellSize, BoundingSphere sphere)
    {
        int size = Mathf.CeilToInt(sphere.radius * 2 / cellSize);
        return Vector3Int.one * size;
    }
}