using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cell
{
    public Vector3Int Position { get; set; }
    public Vector3 WorldPosition { get; set; }
    public GameObject DisplayCell { get; set; }

    public bool IsValid
    {
        get
        {
            return _state;
        }
        set
        {
            _state = value;
        }
    }

    bool _state;
}