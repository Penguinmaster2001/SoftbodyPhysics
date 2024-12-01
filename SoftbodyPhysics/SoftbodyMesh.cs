
using Godot;

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class SoftbodyMesh : MeshInstance3D
{
    private SoftbodyVertex[] _vertices = Array.Empty<SoftbodyVertex>();
    private int[] _meshIndices = Array.Empty<int>();
    private readonly List<int> _softbodyIndices = new();

    private readonly RandomNumberGenerator _rng = new();

    private ImmediateMesh _mesh;

    [Export]
    private float _k;

    [Export]
    private float _vertexMass;

    [Export]
    private float _damping;

    [Export]
    private Mesh _baseMesh;



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
        _vertices = new SoftbodyVertex[meshDataTool.GetVertexCount()];


        for (int vert = 0; vert < meshDataTool.GetVertexCount(); vert++)
        {
            _vertices[vert] = new() {
                Position = meshDataTool.GetVertex(vert),
                UV = meshDataTool.GetVertexUV(vert),
                Normal = meshDataTool.GetVertexNormal(vert),
                Mass = _vertexMass
            };
        }

        GD.Print(_meshIndices.Length);
        GD.Print(surfaceArray[(int)Mesh.ArrayType.Vertex].AsInt32Array().Length);
        GD.Print(meshDataTool.GetVertexCount());

        int springs = 0;
        // Breadth - first search to discover all vertices
        _softbodyIndices.Clear();
        Queue<int> vertexQueue = new();
        bool[] visited = new bool[meshDataTool.GetVertexCount()];
        vertexQueue.Enqueue(0);
        while (vertexQueue.TryDequeue(out int currentVert))// || (currentVert = visited.ToList().IndexOf(false)) > 0)
        {
            visited[currentVert] = true;
            Vector3 currentVertPos = meshDataTool.GetVertex(currentVert);

            List<Spring> vertexSprings = new();
            foreach (int edge in meshDataTool.GetVertexEdges(currentVert))
            {
                int otherVert = meshDataTool.GetEdgeVertex(edge, 0);
                if (otherVert == currentVert)
                {
                    otherVert = meshDataTool.GetEdgeVertex(edge, 1);
                }

                if (!visited[otherVert])
                {
                    visited[otherVert] = true;
                    vertexQueue.Enqueue(otherVert);
                    springs++;

                    vertexSprings.Add(new()
                    {
                        StartVertex = currentVert,
                        EndVertex = otherVert,
                        Stiffness = _k,
                        Dampening = _damping,
                        TargetLength = meshDataTool.GetVertex(otherVert).DistanceTo(currentVertPos)
                    });

                    GD.Print(vertexSprings.Last());
                }
            }

            _softbodyIndices.Add(currentVert);
            SoftbodyVertex vertex = _vertices[currentVert];
            vertex.Springs = vertexSprings.ToArray();
            _vertices[currentVert] = vertex;

            GD.Print(_vertices[currentVert]);
        }

        GD.Print(_softbodyIndices.Count);

        GD.Print(meshDataTool.GetEdgeCount());
        GD.Print(springs);


        _mesh = new();
        Mesh = _mesh;
    }



    public static ArrayMesh MergeVertices(Mesh mesh)
    {
        Godot.Collections.Array surfaceArray = mesh.SurfaceGetArrays(0);
        Vector3[] vertices = surfaceArray[(int)Mesh.ArrayType.Vertex].AsVector3Array();
        Vector2[] uvs = surfaceArray[(int)Mesh.ArrayType.TexUV].AsVector2Array();
        Vector3[] normals = surfaceArray[(int)Mesh.ArrayType.Normal].AsVector3Array();
        int[] indices = surfaceArray[(int)Mesh.ArrayType.Index].AsInt32Array();
        bool[] merged = new bool[vertices.Length];

        List<Vector3> newVertices = new();
        List<Vector2> newUvs = new();
        List<Vector3> newNormals = new();
        int[] newIndices = new int[indices.Length];

        GD.Print(string.Join(",\t", indices));

        for (int i = 0; i < newIndices.Length; i++)
        {
            newIndices[i] = -1;
        }

        for (int index = 0; index < indices.Length; index++)
        {
            int vert = indices[index];
            
            if (merged[vert]) continue;
            merged[vert] = true;
            newIndices[index] = vert;
            
            for (int checkIndex = index + 1; checkIndex < indices.Length; checkIndex++)
            {
                int checkVert = indices[checkIndex];

                if (vertices[vert].DistanceSquaredTo(vertices[checkVert]) < 0.01f)
                {
                    GD.Print($"Merge {vert} with {checkVert}");
                    // GD.Print($"Merge index {index} with {checkIndex}");

                    merged[checkVert] = true;
                    newIndices[checkIndex] = vert;
                }
            }
        }


        GD.Print(string.Join(",\t", newIndices));
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
        GD.Print(string.Join(",\t", newIndices));


        GD.Print(indices.Length);
        GD.Print(newIndices.Count(i => i != -1));


        GD.Print(vertices.Length);
        GD.Print(newVertices.Count);

        var newSurfaceArray = new Godot.Collections.Array();
        newSurfaceArray.Resize((int)Mesh.ArrayType.Max);

        newSurfaceArray[(int)Mesh.ArrayType.Vertex] = newVertices.ToArray();
        newSurfaceArray[(int)Mesh.ArrayType.TexUV] = newUvs.ToArray();
        newSurfaceArray[(int)Mesh.ArrayType.Normal] = newNormals.ToArray();
        newSurfaceArray[(int)Mesh.ArrayType.Index] = newIndices.ToArray();

        ArrayMesh arrayMesh = new();
        arrayMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.Triangles, newSurfaceArray);

        return arrayMesh;
    }



    public override void _PhysicsProcess(double delta)
    {
        UpdateVerts((float)delta);
        UpdateMesh();
    }



    public override void _Input(InputEvent inputEvent)
    {
        if (inputEvent is InputEventMouseButton)
        {
            // int randVert = _rng.RandiRange(0, _vertices.Length - 1);

            // SoftbodyVertex vertex = _vertices[randVert];

            // vertex.Position *= 2.0f * _rng.Randf() + 1.0f;

            // _vertices[randVert] = vertex;
            for (int vert = 0; vert < _vertices.Length; vert++)
            {
                SoftbodyVertex vertex = _vertices[vert];

                vertex.Position *= 2.0f * _rng.Randf() + 1.0f;

                _vertices[vert] = vertex;
            }
        }
    }



    private void UpdateMesh()
    {
        _mesh.ClearSurfaces();
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        for (int vert = 0; vert < _meshIndices.Length; vert++)
        {
            int index = _meshIndices[vert];

            IVertex vertex = _vertices[index];
            
            _mesh.SurfaceSetNormal(vertex.Normal);
            _mesh.SurfaceSetUV(vertex.UV);
            _mesh.SurfaceAddVertex(vertex.Position);
        }
        
        _mesh.SurfaceEnd();
    }



    private void UpdateVerts(float delta)
    {
        for (int vert = 0; vert < _softbodyIndices.Count; vert++)
        {
            int index = _softbodyIndices[vert];
            for (int spring = 0; spring < _vertices[index].Springs.Length; spring++)
            {
                ApplySpring(_vertices[index].Springs[spring]);
            }
        }

        float aveDa = 0.0f;
        for (int vert = 0; vert < _vertices.Length; vert++)
        {
            Vector3 da = _vertices[vert].Force * (delta  / _vertices[vert].Mass);
            _vertices[vert].Position += (_vertices[vert].Velocity + da * 0.5f) * delta;
            _vertices[vert].Velocity += da;
            _vertices[vert].Force = Vector3.Zero;
            if (da.Length() > aveDa)
            {
                aveDa = da.Length();
            }
        }

        if (aveDa > 0.00001f) GD.Print(aveDa);
    }


    private void ApplySpring(Spring spring)
    {
        Vector3 startPos = _vertices[spring.StartVertex].Position;
        Vector3 endPos = _vertices[spring.EndVertex].Position;
        Vector3 springVector = startPos - endPos;
        float springLength = springVector.Length();
        Vector3 springDirection = springVector / springLength;
        float displacement = spring.TargetLength - springLength;
        Vector3 hookesFactor = spring.Stiffness * displacement * springDirection;

        Vector3 startVertDampingVel = -_vertices[spring.StartVertex].Velocity.Project(springDirection);
        Vector3 endVertDampingVel = -_vertices[spring.EndVertex].Velocity.Project(springDirection);

        _vertices[spring.StartVertex].Force += hookesFactor + spring.Dampening * startVertDampingVel;
        _vertices[spring.StartVertex].Force += -hookesFactor + spring.Dampening * endVertDampingVel;
    }
}
