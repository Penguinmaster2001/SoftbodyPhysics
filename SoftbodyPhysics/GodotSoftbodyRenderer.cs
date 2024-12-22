
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class GodotSoftbodyRenderer : MeshInstance3D, ISoftbodyRenderer
{
    private ImmediateMesh _mesh;

    public IVertex[] Vertices { private get; set; }
    public int[] Indices { private get; set; }



    public override void _Ready()
    {
        _mesh = new();
        Mesh = _mesh;
    }



    public override void _Process(double delta)
    {
        UpdateMesh();
    }



    private void UpdateMesh()
    {
        _mesh.ClearSurfaces();
        _mesh.SurfaceBegin(Mesh.PrimitiveType.Triangles);

        for (int i = 0; i < Indices.Length; i += 3)
        {
            Vector3 v0 = DotNetToGodotVector3(Vertices[i + 0].Position);
            Vector3 v1 = DotNetToGodotVector3(Vertices[i + 1].Position);
            Vector3 v2 = DotNetToGodotVector3(Vertices[i + 2].Position);

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



    private static Vector3 DotNetToGodotVector3(System.Numerics.Vector3 vector) => new(vector.X, vector.Y, vector.Z);




    // private void ShowSprings()
    // {
    //     _mesh.SurfaceBegin(Mesh.PrimitiveType.Lines);

    //     for (int springIndex = 0; springIndex < _springs.Length; springIndex++)
    //     {
    //         Spring spring = _springs[springIndex];

    //         SoftbodyVertex startVertex = _state.Vertices[spring.StartVertex];
    //         SoftbodyVertex endVertex = _state.Vertices[spring.EndVertex];
            
    //         float springLength = (startVertex.Position - endVertex.Position).Length();
            
    //         _mesh.SurfaceSetNormal(startVertex.Normal);
    //         _mesh.SurfaceSetUV(startVertex.UV);
    //         _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
    //         _mesh.SurfaceAddVertex(startVertex.Position);

    //         _mesh.SurfaceSetNormal(endVertex.Normal);
    //         _mesh.SurfaceSetUV(endVertex.UV);
    //         _mesh.SurfaceSetColor(new(0.01f * springLength / spring.TargetLength, 0.5f, 0.1f));
    //         _mesh.SurfaceAddVertex(endVertex.Position);
    //     }
        
    //     _mesh.SurfaceEnd();
    // }
}
