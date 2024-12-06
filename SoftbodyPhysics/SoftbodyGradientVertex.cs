
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyGradientVertex
{
    public Vector3 Force;



    public override readonly string ToString()
    {
        return $"SoftbodyGradientVertex. Force: {Force}";
    }
}
