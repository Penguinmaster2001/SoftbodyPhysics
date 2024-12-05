
using Godot;

using System;
using System.Collections.Generic;
using System.Data;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class SoftbodyMesh : MeshInstance3D
{
    private SoftbodyState _state;
    public Spring[] _springs = Array.Empty<Spring>();
    private SoftbodyFace[] _faces = Array.Empty<SoftbodyFace>();
    private int[] _meshIndices = Array.Empty<int>();

    private readonly RandomNumberGenerator _rng = new();

    [Export]
    public SoftbodyEnvironment Environment;

    private ImmediateMesh _mesh;

    [Export]
    private Mesh _baseMesh;

    [Export]
    private Material _springMat;

    [Export]
    private float _youngsModulus;

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

    private const float _epsilon = 0.001f;
    private const float _epsilonSquared = 0.0001f;



    public override void _Ready()
    {
        LoadMeshData();
    }



    private void LoadMeshData()
    {
        ArrayMesh arrayMesh = MergeVertices(_baseMesh);

        Godot.Collections.Array surfaceArray = arrayMesh.SurfaceGetArrays(0);
        _meshIndices = surfaceArray[(int)Mesh.ArrayType.Index].AsInt32Array();


        MeshDataTool meshDataTool = new();
        meshDataTool.CreateFromSurface(arrayMesh, 0);


        _state = new();


        SoftbodyVertex[] vertices = new SoftbodyVertex[meshDataTool.GetVertexCount()];
        for (int vert = 0; vert < meshDataTool.GetVertexCount(); vert++)
        {
            vertices[vert] = new() {
                Position = meshDataTool.GetVertex(vert),
                UV = meshDataTool.GetVertexUV(vert),
                Normal = meshDataTool.GetVertexNormal(vert),
                InitialArea = 0.0f
            };
        }
        _state.Vertices = vertices;


        SoftbodyFace[] faces = new SoftbodyFace[meshDataTool.GetFaceCount()];
        for (int face = 0; face < meshDataTool.GetFaceCount(); face++)
        {
            faces[face] = new() {
                V0 = meshDataTool.GetFaceVertex(face, 0),
                V1 = meshDataTool.GetFaceVertex(face, 1),
                V2 = meshDataTool.GetFaceVertex(face, 2)
            };
            Vector3 v0 = vertices[faces[face].V0].Position;
            Vector3 v1 = vertices[faces[face].V1].Position;
            Vector3 v2 = vertices[faces[face].V2].Position;

            float faceAreaPerVert = 0.3333f * 0.5f * (v2 - v0).Cross(v1 - v0).Length();

            vertices[faces[face].V0].InitialArea += faceAreaPerVert;
            vertices[faces[face].V1].InitialArea += faceAreaPerVert;
            vertices[faces[face].V2].InitialArea += faceAreaPerVert;
        }
        _faces = faces;
        

        Spring[] springs = new Spring[meshDataTool.GetEdgeCount()];
        for (int edge = 0; edge < meshDataTool.GetEdgeCount(); edge++)
        {
            int startVert = meshDataTool.GetEdgeVertex(edge, 0);
            int endVert = meshDataTool.GetEdgeVertex(edge, 1);

            float startLength = meshDataTool.GetVertex(startVert).DistanceTo(meshDataTool.GetVertex(endVert));
            float springArea = _state.Vertices[startVert].InitialArea + _state.Vertices[endVert].InitialArea;

            springs[edge] = new() {
                    StartVertex = startVert,
                    EndVertex = endVert,
                    LengthAreaCoefficient = springArea / startLength,
                    Dampening = _damping,
                    TargetLength = startLength
                };
        }
        _springs = springs;


        _mesh = new();
        Mesh = _mesh;

        _startVolume = CalculateVolume();
    }



    public static ArrayMesh MergeVertices(Mesh mesh)
    {
        // Get data from mesh
        Godot.Collections.Array surfaceArray = mesh.SurfaceGetArrays(0);
        Vector3[] vertices = surfaceArray[(int)Mesh.ArrayType.Vertex].AsVector3Array();
        Vector2[] uvs = surfaceArray[(int)Mesh.ArrayType.TexUV].AsVector2Array();
        Vector3[] normals = surfaceArray[(int)Mesh.ArrayType.Normal].AsVector3Array();
        int[] indices = surfaceArray[(int)Mesh.ArrayType.Index].AsInt32Array();
        bool[] merged = new bool[vertices.Length];

        // Lists for new mesh
        // These lists may be smaller
        List<Vector3> newVertices = new();
        List<Vector2> newUvs = new();
        List<Vector3> newNormals = new();
        // Indices array is exactly the same length
        int[] newIndices = new int[indices.Length];


        // Iterate through the indices and ignore indices that are close together
        for (int index = 0; index < indices.Length; index++)
        {
            int vert = indices[index];
            
            if (merged[vert]) continue;
            merged[vert] = true;
            newIndices[index] = vert;
            
            for (int checkIndex = index + 1; checkIndex < indices.Length; checkIndex++)
            {
                int checkVert = indices[checkIndex];

                if (vertices[vert].DistanceSquaredTo(vertices[checkVert]) < _epsilonSquared)
                {
                    merged[checkVert] = true;
                    newIndices[checkIndex] = vert;
                }
            }
        }


        // Remake the index - vertex mapping and fill vertex, uv, and normal lists
        int newVert = 0;
        int updatedVerts = 0;
        for (int oldIndex = 0; oldIndex < newIndices.Length && updatedVerts < newIndices.Length; oldIndex++)
        {
            int oldVert = newIndices[oldIndex];
            if (oldVert < newVert) continue;
            
            newVertices.Add(vertices[oldVert]);
            newUvs.Add(uvs[oldVert]);
            newNormals.Add(normals[oldVert]);
            
            for (int i = 0; i < newIndices.Length; i++)
            {
                if (newIndices[i] == oldVert)
                {
                    newIndices[i] = newVert;
                    updatedVerts++;
                }
            }

            newVert++;
        }


        // Create a new arrayMesh
        Godot.Collections.Array newSurfaceArray = new();
        newSurfaceArray.Resize((int)Mesh.ArrayType.Max);

        newSurfaceArray[(int)Mesh.ArrayType.Vertex] = newVertices.ToArray();
        newSurfaceArray[(int)Mesh.ArrayType.TexUV] = newUvs.ToArray();
        newSurfaceArray[(int)Mesh.ArrayType.Normal] = newNormals.ToArray();
        newSurfaceArray[(int)Mesh.ArrayType.Index] = newIndices;

        ArrayMesh arrayMesh = new();
        arrayMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, newSurfaceArray);

        return arrayMesh;
    }



    bool error = false;
    bool render = true;
    bool doRk4 = false;
    public override void _PhysicsProcess(double delta)
    {
        if (error) return;
        
        float dt = (float)(_timeSpeed * delta / _subSteps);
        for (int subStep = 0; subStep < _subSteps; subStep++)
        {
            if (doRk4) UpdateRk4(dt);
            else UpdateEuler(dt);
        }

        _mesh.ClearSurfaces();
        if (render) UpdateMesh(_state.Vertices);
        else ShowSprings();
    }



    private void UpdateEuler(float dt)
    {
        SoftbodyState k1 = CalculateDerivatives(_state);
        _state += dt * k1;
    }



    private void UpdateRk4(float dt)
    {
        SoftbodyState k1 = CalculateDerivatives(_state);
        SoftbodyState k2 = CalculateDerivatives(_state + (0.5f * dt * k1));
        SoftbodyState k3 = CalculateDerivatives(_state + (0.5f * dt * k2));
        SoftbodyState k4 = CalculateDerivatives(_state + (dt * k3));

        _state += dt / 6.0f * (k1 + (2.0f * k2) + (2.0f * k3) + k4);
    }



    private SoftbodyState CalculateDerivatives(SoftbodyState state)
    {
        SoftbodyState newState = state;
        CalculateSprings(ref newState);
        CalculateInternalPressure(ref newState);
        CalculateEnvironment(ref newState);
        
        return newState;
    }



    public override void _Input(InputEvent inputEvent)
    {
        if (inputEvent is InputEventKey keyEvent && keyEvent.IsReleased())
        {
            switch (keyEvent.Keycode)
            {
                case Key.R:
                    error = false;
                    LoadMeshData();
                    break;
                case Key.P:
                    render = !render;
                    break;
                case Key.E:
                    doRk4 = !doRk4;
                    GD.Print($"doRk4: {doRk4}");
                    break;
            }
        }
    }



    private void CalculateSprings(ref SoftbodyState state)
    {
        for (int springIndex = 0; springIndex < _springs.Length; springIndex++)
        {
            Spring spring = _springs[springIndex];

            spring.Dampening = _damping;


            SoftbodyVertex startVertex = state.Vertices[spring.StartVertex];
            SoftbodyVertex endVertex = state.Vertices[spring.EndVertex];
            
            Vector3 springVector = startVertex.Position - endVertex.Position;
            float springLength = springVector.Length();
            if (springLength < _epsilon)
            {
                springLength = _epsilon;
            }

            Vector3 springDirection = springVector / springLength;
            Vector3 hookesFactor = spring.LengthAreaCoefficient * _youngsModulus * (spring.TargetLength - springLength) * springDirection;

            Vector3 startVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -startVertex.Velocity.Project(springDirection);
            Vector3 endVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -endVertex.Velocity.Project(springDirection);

            startVertex.Force += LimitInfiniteLength(hookesFactor + spring.Dampening * startVertDampingVel, _maxForce);
            endVertex.Force += LimitInfiniteLength(-hookesFactor + spring.Dampening * endVertDampingVel, _maxForce);

            state.Vertices[spring.StartVertex] = startVertex;
            state.Vertices[spring.EndVertex] = endVertex;
        }
    }



    private void CalculateInternalPressure(ref SoftbodyState state)
    {
        float newVolume = CalculateVolume();
        state.Pressure = _mols * _pressureConstant * state.Temperature / newVolume;

        state.SurfaceArea = 0.0f;
        
        for (int faceIndex = 0; faceIndex < _faces.Length; faceIndex++)
        {
            SoftbodyFace face = _faces[faceIndex];
            Vector3 v0 = state.Vertices[face.V0].Position;
            Vector3 v1 = state.Vertices[face.V1].Position;
            Vector3 v2 = state.Vertices[face.V2].Position;

            Vector3 aveVel = 0.3333f * 
                            (state.Vertices[face.V0].Velocity +
                             state.Vertices[face.V1].Velocity +
                             state.Vertices[face.V2].Velocity);
            
            // The length of this is twice the area of the tri
            // The direction is the normal of the tri
            Vector3 cross = (v2 - v0).Cross(v1 - v0);
            float area = 0.5f * cross.Length();
            state.SurfaceArea += area;

            Vector3 dragForce = aveVel * (_dragMultiplier * Environment.AirDensity * area * -aveVel.Length());

            Vector3 pressureForce = cross * ((0.5f * state.Pressure) - Environment.AirPressure);
            Vector3 forcePerVertex = LimitInfiniteLength(0.3333f * (pressureForce + dragForce), _maxForce);
            state.Vertices[face.V0].Force += forcePerVertex;
            state.Vertices[face.V1].Force += forcePerVertex;
            state.Vertices[face.V2].Force += forcePerVertex;
        }


        state.ThermalFlux = Mathf.Clamp(_skinThermalConductivity * state.SurfaceArea * (Environment.AirTemperature - state.Temperature), -_maxForce, _maxForce);
        state.Temperature = Mathf.Clamp(state.Temperature + (0.66667f * (-state.Pressure * (newVolume - state.Volume)) / (_mols * _pressureConstant)), 0.0f, _maxForce);
        state.Volume = newVolume;
    }



    private float CalculateVolume()
    {
        float totalVolume = 0.0f;
        for (int faceIndex = 0; faceIndex < _faces.Length; faceIndex++)
        {
            SoftbodyFace face = _faces[faceIndex];
            Vector3 v0 = _state.Vertices[face.V0].Position;
            Vector3 v1 = _state.Vertices[face.V1].Position;
            Vector3 v2 = _state.Vertices[face.V2].Position;
            totalVolume += v2.Dot(v1.Cross(v0));
        }

        return Mathf.Max(0.16667f * totalVolume, _epsilon);
    }



    private void CalculateEnvironment(ref SoftbodyState state)
    {
        state.GasMass = _mols * _gasMolDensity;

        for (int vert = 0; vert < state.Vertices.Length; vert++)
        {
            // Gravity per vertex
            state.Vertices[vert].Force += ((state.Vertices[vert].InitialArea * _initialSkinAreaMass) + state.GasMass / state.Vertices.Length) * Environment.Gravity;

            // Buoyancy per vertex
            state.Vertices[vert].Force += Environment.AirDensity * state.Volume / -state.Vertices.Length * Environment.Gravity;

            if (ToGlobal(state.Vertices[vert].Position).Y < 0.0f)
            {
                state.Vertices[vert].Force *= 0.5f;
                state.Vertices[vert].Force += 1.5f * state.Vertices[vert].Force.Y * state.Vertices[vert].InitialArea * ToGlobal(state.Vertices[vert].Position).Y * Vector3.Up;
                state.Vertices[vert].Position = ToLocal(ToGlobal(state.Vertices[vert].Position) * new Vector3(1.0f, 0.0f, 1.0f));
            }
        }
    }



    private void UpdateMesh(SoftbodyVertex[] vertices)
    {
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        for (int faceIndex = 0; faceIndex < _faces.Length; faceIndex++)
        {
            SoftbodyFace face = _faces[faceIndex];
            Vector3 v0 = vertices[face.V0].Position;
            Vector3 v1 = vertices[face.V1].Position;
            Vector3 v2 = vertices[face.V2].Position;

            Vector3 normal = (v2 - v0).Cross(v1 - v0).Normalized();

            _mesh.SurfaceSetNormal(normal);
            _mesh.SurfaceAddVertex(v0);

            _mesh.SurfaceSetNormal(normal);
            _mesh.SurfaceAddVertex(v1);

            _mesh.SurfaceSetNormal(normal);
            _mesh.SurfaceAddVertex(v2);
        }
        
        _mesh.SurfaceEnd();
    }



    private void ShowSprings()
    {
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Lines);

        for (int springIndex = 0; springIndex < _springs.Length; springIndex++)
        {
            Spring spring = _springs[springIndex];

            SoftbodyVertex startVertex = _state.Vertices[spring.StartVertex];
            SoftbodyVertex endVertex = _state.Vertices[spring.EndVertex];
            
            float springLength = (startVertex.Position - endVertex.Position).Length();
            
            _mesh.SurfaceSetNormal(startVertex.Normal);
            _mesh.SurfaceSetUV(startVertex.UV);
            _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
            _mesh.SurfaceAddVertex(startVertex.Position);

            _mesh.SurfaceSetNormal(endVertex.Normal);
            _mesh.SurfaceSetUV(endVertex.UV);
            _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
            _mesh.SurfaceAddVertex(endVertex.Position);
        }
        
        _mesh.SurfaceEnd();
    }



    private static Vector3 LimitInfiniteLength(Vector3 vector, float length)
    {
        if (!vector.IsFinite())
        {
            vector = new() {
                X = float.IsNegativeInfinity(vector.X) ? -1.0f : (float.IsPositiveInfinity(vector.X) ? 1.0f : 0.0f),
                Y = float.IsNegativeInfinity(vector.Y) ? -1.0f : (float.IsPositiveInfinity(vector.Y) ? 1.0f : 0.0f),
                Z = float.IsNegativeInfinity(vector.Z) ? -1.0f : (float.IsPositiveInfinity(vector.Z) ? 1.0f : 0.0f)
            };
        }

        return vector.LimitLength(length);
    }
}
