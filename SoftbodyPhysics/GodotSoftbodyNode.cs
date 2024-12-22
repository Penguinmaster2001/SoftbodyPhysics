
using Godot;

using System;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class GodotSoftbodyNode : Node3D
{
    private GodotSoftbodyRenderer _renderer;
    private Softbody _softbody;

    [Export]
    public SoftbodyEnvironment Environment;

    [Export]
    private Mesh _baseMesh;

    [Export]
    private Material _springMat;

    [Export]
    private float _youngsModulus;

    [Export]
    private float _preCompress;

    [Export]
    private float _initialSkinAreaMass;

    [Export]
    private float _damping;

    [Export]
    private float _pressureConstant;

    [Export]
    private float _startVolume;

    [Export]
    private float _mols;

    [Export]
    private float _skinThermalConductivity;

    [Export]
    private float _gasMolDensity;

    [Export]
    private float _dragMultiplier;

    [Export]
    private float _maxForce;

    [Export]
    private int _subSteps;

    [Export]
    private double _timeSpeed;



    public override void _Ready()
    {
        _renderer = GetChild<GodotSoftbodyRenderer>(0);
        _softbody = SoftbodyMeshLoader.LoadMesh(_baseMesh);
    }



    public override void _Process(double delta)
    {

    }
}
