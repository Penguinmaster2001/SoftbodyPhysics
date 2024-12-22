
using Godot;

using System.Collections.Generic;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public static class SoftbodyMeshLoader
{
    public static Softbody LoadMesh(Mesh mesh, float mergeDistance = 0.001f) => LoadMeshData(MergeVertices(mesh, mergeDistance));



    public static Softbody LoadMeshData(MeshDataTool meshData)
    {
        SoftbodyVertex[] vertices = new SoftbodyVertex[meshData.GetVertexCount()];
        for (int vert = 0; vert < meshData.GetVertexCount(); vert++)
        {
            vertices[vert] = new() {
                Position = meshData.GetVertex(vert),
                UV = meshData.GetVertexUV(vert),
                Normal = meshData.GetVertexNormal(vert),
                InitialArea = 0.0f
            };
        }


        SoftbodyFace[] faces = new SoftbodyFace[meshData.GetFaceCount()];
        for (int face = 0; face < meshData.GetFaceCount(); face++)
        {
            faces[face] = new() {
                V0 = meshData.GetFaceVertex(face, 0),
                V1 = meshData.GetFaceVertex(face, 1),
                V2 = meshData.GetFaceVertex(face, 2)
            };
            Vector3 v0 = vertices[faces[face].V0].Position;
            Vector3 v1 = vertices[faces[face].V1].Position;
            Vector3 v2 = vertices[faces[face].V2].Position;

            float faceAreaPerVert = 0.3333f * 0.5f * (v2 - v0).Cross(v1 - v0).Length();

            vertices[faces[face].V0].InitialArea += faceAreaPerVert;
            vertices[faces[face].V1].InitialArea += faceAreaPerVert;
            vertices[faces[face].V2].InitialArea += faceAreaPerVert;
        }
        

        Spring[] springs = new Spring[meshData.GetEdgeCount()];
        for (int edge = 0; edge < meshData.GetEdgeCount(); edge++)
        {
            int startVert = meshData.GetEdgeVertex(edge, 0);
            int endVert = meshData.GetEdgeVertex(edge, 1);

            float startLength = meshData.GetVertex(startVert).DistanceTo(meshData.GetVertex(endVert));
            float springArea = vertices[startVert].InitialArea + vertices[endVert].InitialArea;

            springs[edge] = new() {
                    StartVertex = startVert,
                    EndVertex = endVert,
                    LengthAreaCoefficient = springArea / startLength,
                    TargetLength = startLength
                };
        }
        

        return new() {
            Springs = springs,
            Faces = faces,
            Indices = new int[meshData.GetFaceCount() * 3],

            StartVolume = CalculateVolume(meshData),

            State = new() {
                Temperature = 0.0f,

                Vertices = vertices
            },
        };
    }



    public static MeshDataTool MergeVertices(Mesh mesh, float distance)
    {
        float distanceSquared = distance * distance;


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

                if (vertices[vert].DistanceSquaredTo(vertices[checkVert]) < distanceSquared)
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

        MeshDataTool meshData = new();
        meshData.CreateFromSurface(arrayMesh, 0);

        return meshData;
    }



    public static float CalculateVolume(MeshDataTool meshData)
    {
        float totalVolume = 0.0f;
        for (int face = 0; face < meshData.GetFaceCount(); face++)
        {
            Vector3 v0 = meshData.GetVertex(meshData.GetFaceVertex(face, 0));
            Vector3 v1 = meshData.GetVertex(meshData.GetFaceVertex(face, 1));
            Vector3 v2 = meshData.GetVertex(meshData.GetFaceVertex(face, 0));
            totalVolume += v2.Dot(v1.Cross(v0));
        }

        return Mathf.Max(0.16667f * totalVolume, 0.001f);
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
