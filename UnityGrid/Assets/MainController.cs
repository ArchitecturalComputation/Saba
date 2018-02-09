using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainController : MonoBehaviour
{
    [SerializeField]
    private Mesh _cellGeometry;

    [SerializeField]
    private Material _cellMaterial;

    private float _cellSize;
    private ChunkSubdivision _chunkDivision;
    private List<Material> _chunkMaterials = new List<Material>();

    void Start()
    {
        _cellSize = 0.5f;
        var sphere = new BoundingSphere(new Vector3(20, 30, 40), 10);
        int chunkCount = 10;
        _chunkDivision = new ChunkSubdivision(_cellSize, sphere, chunkCount);

        StartCoroutine(AddChunk());
    }


    void Update()
    {
        if (_chunkDivision == null) return;

        int i = 0;
        foreach (var chunk in _chunkDivision.Chunks)
        {
            if (_chunkMaterials.Count - 1 < i)
            {
                var newMaterial = new Material(_cellMaterial);
                newMaterial.color = Random.ColorHSV(0, 0.5f, 0.8f, 0.8f, 0.8f, 0.8f);
                _chunkMaterials.Add(newMaterial);
            }

            var material = _chunkMaterials[i++];

            foreach (var cell in chunk)
            {
                var matrix = Matrix4x4.TRS(cell.WorldPosition, Quaternion.identity, Vector3.one * _cellSize * 0.90f);
                Graphics.DrawMesh(_cellGeometry, matrix, material, 0);
            }
        }
    }

    IEnumerator AddChunk()
    {
        while (!_chunkDivision.AllCellsAdded())
        {
            _chunkDivision.CreateChunks(10);
            yield return new WaitForSeconds(0.25f);
        }
    }

}
