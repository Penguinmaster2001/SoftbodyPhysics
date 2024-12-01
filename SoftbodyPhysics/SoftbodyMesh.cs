
using Godot;

using System.Collections.Generic;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class SoftbodyMesh : MeshInstance3D
{
    private readonly List<Vector3> _vertices = new();
    private readonly List<List<int>> _vertexNeighbors = new();
    private readonly List<Vector3> _velocities = new();
    private readonly List<Vector2> _uvs = new();
    private readonly List<Vector3> _normals = new();
    private readonly List<int> _indices = new();
    private readonly RandomNumberGenerator _rng = new();

    private ImmediateMesh _mesh;

    [Export]
    private float _k;

    [Export]
    private float _radius;

    [Export]
    private float _pointMasses;

    [Export]
    private float _damping;



    public override void _Ready()
    {
        Mesh = new ImmediateMesh();
        _mesh = Mesh as ImmediateMesh;

        GenerateSphere();
    }



    public override void _PhysicsProcess(double delta)
    {
        UpdateVerts((float)delta);
        UpdateMesh();
    }



    public override void _Input(InputEvent @event)
    {
        if (@event is InputEventMouseButton)
        {
            int randVert = _rng.RandiRange(0, _vertices.Count - 1);

            _vertices[randVert] *= _rng.Randf() + 0.5f;
        }
    }



    private void UpdateMesh()
    {
        _mesh.ClearSurfaces();
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        for (int vert = 0; vert < _indices.Count; vert++)
        {
            int index = _indices[vert];
            _mesh.SurfaceSetNormal(_normals[index]);
            _mesh.SurfaceSetUV(_uvs[index]);
            _mesh.SurfaceAddVertex(_vertices[index]);
        }
        
        _mesh.SurfaceEnd();
    }



    private void UpdateVerts(float delta)
    {
        float aveDa = 0.0f;
        for (int vert = 0; vert < _vertices.Count; vert++)
        {
            Vector3 position = _vertices[vert];

            for (int neighbor = 0; neighbor < _vertexNeighbors[vert].Count; neighbor++)
            {
                int neighborVert = _vertexNeighbors[vert][neighbor];
            }

            float vert_radius = position.Length();

            Vector3 target_pos = _vertices[vert] * _radius / vert_radius;

            Vector3 hookesFactor = _k * (target_pos - position);

            Vector3 dampedFactor = _damping * -_velocities[vert];

            Vector3 da = (hookesFactor + dampedFactor) * (delta  / _pointMasses);

            _vertices[vert] += (_velocities[vert] + da * 0.5f) * delta;

            _velocities[vert] += da;

            aveDa += da.Length();
        }

        GD.Print(aveDa / _vertices.Count);
    }



    private void GenerateSphere()
    {
        int rings = 50;
        int radialSegments = 50;
        float radius = 1;

        int thisRow = 0;
        int prevRow = 0;
        int point = 0;

        for (int i = 0; i < rings; i++)
        {
            float v = (float)i / rings;
            (float w, float y) = Mathf.SinCos(Mathf.Pi * v);

            for (int j = 0; j <= radialSegments; j++)
            {
                float u = (float)j / radialSegments;
                (float x, float z) = Mathf.SinCos(2.0f * Mathf.Pi * u);

                Vector3 vert = new(x * radius * w, y * radius, z * radius * w);
                _vertices.Add(vert);
                _velocities.Add(Vector3.Zero);
                _vertexNeighbors.Add(new());
                _normals.Add(vert.Normalized());
                _uvs.Add(new(u, v));
                point++;

                if (i > 0 && j > 0)
                {
                    _indices.Add(prevRow + j - 1);
                    _indices.Add(prevRow + j);
                    _indices.Add(thisRow + j - 1);

                    _vertexNeighbors[prevRow + j].Add(prevRow + j - 1);
                    _vertexNeighbors[prevRow + j].Add(thisRow + j - 1);

                    _indices.Add(prevRow + j);
                    _indices.Add(thisRow + j);
                    _indices.Add(thisRow + j - 1);

                    _vertexNeighbors[thisRow + j].Add(prevRow + j);
                    _vertexNeighbors[thisRow + j].Add(thisRow + j - 1);

                }
            }

            prevRow = thisRow;
            thisRow = point;
        }
    }
}
