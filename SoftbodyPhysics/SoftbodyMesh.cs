
using Godot;

using System;
using System.Collections.Generic;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class SoftbodyMesh : MeshInstance3D
{
    private SoftbodyState _state;
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
    private float _k;

    [Export]
    private float _initialSkinAreaMass;

    [Export]
    private float _damping;

    [Export]
    private float _pressureConstant;

    [Export]
    private float _pressure;

    [Export]
    private float _startVolume;

    [Export]
    private float _volume;

    [Export]
    private float _surfaceArea;

    [Export]
    private float _mols;

    [Export]
    private float _temperature;

    [Export]
    private float _skinThermalConductivity;

    [Export]
    private float _gasMolDensity;

    [Export]
    private float _gasMass;

    [Export]
    private float _dragMultiplier;

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


        SoftbodyVertex[] vertices = new SoftbodyVertex[meshDataTool.GetVertexCount()];
        for (int vert = 0; vert < meshDataTool.GetVertexCount(); vert++)
        {
            vertices[vert] = new() {
                Position = meshDataTool.GetVertex(vert),
                UV = meshDataTool.GetVertexUV(vert),
                Normal = meshDataTool.GetVertexNormal(vert),
                Mass = 0.0f
            };
        }
        _state.Vertices = vertices;
        

        Spring[] springs = new Spring[meshDataTool.GetEdgeCount()];
        for (int edge = 0; edge < meshDataTool.GetEdgeCount(); edge++)
        {
            int startVert = meshDataTool.GetEdgeVertex(edge, 0);
            int endVert = meshDataTool.GetEdgeVertex(edge, 1);

            springs[edge] = new() {
                    StartVertex = startVert,
                    EndVertex = endVert,
                    Stiffness = _k,
                    Dampening = _damping,
                    TargetLength = meshDataTool.GetVertex(startVert)
                                               .DistanceTo(meshDataTool.GetVertex(endVert))
                };
        }
        _state.Springs = springs;


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

            float faceMassPerVert = 0.3333f * _initialSkinAreaMass * 0.5f * (v2 - v0).Cross(v1 - v0).Length();

            vertices[faces[face].V0].Mass += faceMassPerVert;
            vertices[faces[face].V1].Mass += faceMassPerVert;
            vertices[faces[face].V2].Mass += faceMassPerVert;
        }
        _faces = faces;


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
    bool render = false;
    public override void _PhysicsProcess(double delta)
    {
        if (error) return;
        GD.Print("frame");

        for (int subStep = 0; subStep < _subSteps; subStep++)
        {
            UpdateRk4((float)(_timeSpeed * delta / _subSteps));
        }

        if (render) UpdateMesh(_state.Vertices);
        else ShowSprings();
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
        CalculateSprings(newState);
        CalculateInternalPressure(newState);
        CalculateEnvironment(newState);
        
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
            }
        }
    }



    private void CalculateSprings(SoftbodyState state)
    {
        for (int springIndex = 0; springIndex < state.Springs.Length; springIndex++)
        {
            Spring spring = state.Springs[springIndex];

            spring.Stiffness = _k;
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
            Vector3 hookesFactor = spring.Stiffness * (spring.TargetLength - springLength) * springDirection;

            Vector3 startVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -startVertex.Velocity.Project(springDirection);
            Vector3 endVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -endVertex.Velocity.Project(springDirection);

            startVertex.Force += hookesFactor + spring.Dampening * startVertDampingVel;
            endVertex.Force += -hookesFactor + spring.Dampening * endVertDampingVel;

            state.Vertices[spring.StartVertex] = startVertex;
            state.Vertices[spring.EndVertex] = endVertex;

            if (Mathf.IsNaN(state.Vertices[spring.StartVertex].Force.Length()))
            {
                GD.Print("Spring start");
                GD.Print($"sv: {springVector}\tsl: {springLength}\tsd: {springDirection}\thf: {hookesFactor}\tsvdv: {startVertDampingVel}\tsvf: {startVertex.Force}\tsvv: {startVertex.Velocity}\tnsvv: {-startVertex.Velocity}");

            }
            if (Mathf.IsNaN(state.Vertices[spring.EndVertex].Force.Length())) GD.Print("Spring end");

            _mesh.SurfaceSetNormal(startVertex.Normal);
            _mesh.SurfaceSetUV(startVertex.UV);
            _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
            _mesh.SurfaceAddVertex(startVertex.Position);

            _mesh.SurfaceSetNormal(endVertex.Normal);
            _mesh.SurfaceSetUV(endVertex.UV);
            _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
            _mesh.SurfaceAddVertex(endVertex.Position);
        }
    }



    private void CalculateInternalPressure(SoftbodyState state)
    {
        float newVolume = CalculateVolume();
        _pressure = _mols * _pressureConstant * _temperature / newVolume;

        _surfaceArea = 0.0f;
        
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
            _surfaceArea += area;

            Vector3 dragForce = aveVel * (_dragMultiplier * Environment.AirDensity * area * -aveVel.Length());

            Vector3 pressureForce = cross * (0.5f * _pressure);
            state.Vertices[face.V0].Force += 0.3333f * (pressureForce + dragForce);
            state.Vertices[face.V1].Force += 0.3333f * (pressureForce + dragForce);
            state.Vertices[face.V2].Force += 0.3333f * (pressureForce + dragForce);


            if (Mathf.IsNaN(state.Vertices[face.V0].Force.Length()))
            {
                GD.Print("Pressure v0");
                GD.Print($"mols: {_mols}");
                GD.Print($"_pressureConstant: {_pressureConstant}");
                GD.Print($"_temperature: {_temperature}");
                GD.Print($"newVolume: {newVolume}");
                GD.Print($"_pressure: {_pressure}");
                GD.Print($"cross: {cross}");
                GD.Print($"pressureForce: {pressureForce}");
                GD.Print($"dragForce: {dragForce}");
                GD.Print($"aveVel: {aveVel}");
                GD.Print($"area: {area}");
            }
            
            if (Mathf.IsNaN(state.Vertices[face.V1].Force.Length())) GD.Print("Pressure v1");
            if (Mathf.IsNaN(state.Vertices[face.V2].Force.Length())) GD.Print("Pressure v2");
        }


        state.ThermalFlux = _skinThermalConductivity * _surfaceArea * (Environment.AirTemperature - _temperature);
        state.Temperature += 0.66667f * (-_pressure * (newVolume - _volume)) / (_mols * _pressureConstant);
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

            if (Mathf.IsNaN(totalVolume))
            {
                GD.Print($"\n\n\nTotal volume face index: {faceIndex}");
                GD.Print(_state.Vertices[face.V0].Position);
                GD.Print(_state.Vertices[face.V1].Position);
                GD.Print(_state.Vertices[face.V2].Position);
                GD.Print(v1.Cross(v0));
                GD.Print(v2.Dot(v1.Cross(v0)));
            }
            if (Mathf.IsNaN(_state.Vertices[face.V0].Position.Length())) GD.Print($"Volume v0: {face.V0}");
            if (Mathf.IsNaN(_state.Vertices[face.V1].Position.Length())) GD.Print($"Volume v1: {face.V1}");
            if (Mathf.IsNaN(_state.Vertices[face.V2].Position.Length())) GD.Print($"Volume v2: {face.V2}");
        }

        return Mathf.Max(0.16667f * totalVolume, _epsilon);
    }



    private void CalculateEnvironment(SoftbodyState state)
    {
        for (int vert = 0; vert < _vertices.Length; vert++)
        {
            if (Mathf.IsNaN(_vertices[vert].Force.Length()))
            {
                GD.Print("Update Verts");
                error = true;
            }
            
            // Gravity per vertex
            _vertices[vert].Force += (_vertices[vert].Mass + _gasMass / _vertices.Length) * Environment.Gravity;

            // Buoyancy per vertex
            _vertices[vert].Force += Environment.AirDensity * _volume / -_vertices.Length * Environment.Gravity;

            if (ToGlobal(_vertices[vert].Position).Y < 0.0f)
            {
                _vertices[vert].Force *= 0.5f;
                _vertices[vert].Force += 1.5f * _vertices[vert].Force.Y *_vertices[vert].Mass * ToGlobal(_vertices[vert].Position).Y * Vector3.Up;
            }

            // Vector3 da = ;
            // _vertices[vert].Position += (_vertices[vert].Velocity + da * 0.5f) * delta;
            // _vertices[vert].Velocity += da;
            (_vertices[vert].Position, _vertices[vert].Velocity) = RK4Integration(_vertices[vert].Position, _vertices[vert].Velocity);

            _vertices[vert].Force = Vector3.Zero;

            if (ToGlobal(_vertices[vert].Position).Y < 0.0f)
            {
                _vertices[vert].Position = ToLocal(ToGlobal(_vertices[vert].Position) * new Vector3(1.0f, 0.0f, 1.0f));
            }


            (Vector3, Vector3) RK4Integration(Vector3 pos, Vector3 vel)
            {
                (Vector3 k1Pos, Vector3 k1Vel) = Derivatives(pos, vel);
                (Vector3 k2Pos, Vector3 k2Vel) = Derivatives(pos + (0.5f * delta * k1Pos), vel + (0.5f * delta * k1Vel));
                (Vector3 k3Pos, Vector3 k3Vel) = Derivatives(pos + (0.5f * delta * k2Pos), vel + (0.5f * delta * k2Vel));
                (Vector3 k4Pos, Vector3 k4Vel) = Derivatives(pos + (delta * k3Pos), vel + (delta * k3Vel));
                pos += (delta / 6.0f) * (k1Pos + (2.0f * k2Pos) + (2.0f * k3Pos) + k4Pos);
                vel += (delta / 6.0f) * (k1Vel + (2.0f * k2Vel) + (2.0f * k3Vel) + k4Vel);
                return (pos, vel);
            }


            // Returns derivatives of position and velocity as (velocity, acceleration)
            (Vector3, Vector3) Derivatives(Vector3 position, Vector3 velocity)
            {
                Vector3 acc = _vertices[vert].Force / _vertices[vert].Mass;
                return (velocity, acc);
            }
        }
    }



    private void UpdateMesh(SoftbodyVertex[] vertices)
    {
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        for (int vert = 0; vert < _meshIndices.Length; vert++)
        {
            int index = _meshIndices[vert];

            IVertex vertex = vertices[index];
            
            _mesh.SurfaceSetNormal(vertex.Normal);
            _mesh.SurfaceSetUV(vertex.UV);
            _mesh.SurfaceAddVertex(vertex.Position);
        }
        
        _mesh.SurfaceEnd();
    }

    private void ShowSprings()
    {

    }


    private void ApplySpring(SoftbodyVertex[] inVector, SoftbodyVertex[] outVector, float dt, Spring spring)
    {
        spring.Stiffness = _k;
        spring.Dampening = _damping;


        SoftbodyVertex startVertex = inVector[spring.StartVertex];
        SoftbodyVertex endVertex = inVector[spring.EndVertex];
        
        Vector3 springVector = startVertex.Position - endVertex.Position;
        float springLength = springVector.Length();
        if (springLength < _epsilon)
        {
            springLength = _epsilon;
        }

        Vector3 springDirection = springVector / springLength;
        Vector3 hookesFactor = spring.Stiffness * (spring.TargetLength - springLength) * springDirection;

        Vector3 startVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -startVertex.Velocity.Project(springDirection);
        Vector3 endVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -endVertex.Velocity.Project(springDirection);

        startVertex.Force += hookesFactor + spring.Dampening * startVertDampingVel;
        endVertex.Force += -hookesFactor + spring.Dampening * endVertDampingVel;

        outVector[spring.StartVertex] = startVertex;
        outVector[spring.EndVertex] = endVertex;

        if (Mathf.IsNaN(outVector[spring.StartVertex].Force.Length()))
        {
            GD.Print("Spring start");
            GD.Print($"sv: {springVector}\tsl: {springLength}\tsd: {springDirection}\thf: {hookesFactor}\tsvdv: {startVertDampingVel}\tsvf: {startVertex.Force}\tsvv: {startVertex.Velocity}\tnsvv: {-startVertex.Velocity}");

        }
        if (Mathf.IsNaN(outVector[spring.EndVertex].Force.Length())) GD.Print("Spring end");

        _mesh.SurfaceSetNormal(startVertex.Normal);
        _mesh.SurfaceSetUV(startVertex.UV);
        _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
        _mesh.SurfaceAddVertex(startVertex.Position);

        _mesh.SurfaceSetNormal(endVertex.Normal);
        _mesh.SurfaceSetUV(endVertex.UV);
        _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
        _mesh.SurfaceAddVertex(endVertex.Position);
    }
}
