
namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct Spring
{
    public int StartVertex;
    public int EndVertex;
    public float Stiffness;
    public float Dampening;
    public float TargetLength;



    public override readonly string ToString()
    {
        return $"Spring from {StartVertex} to {EndVertex}. Stiffness: {Stiffness}. Dampening: {Dampening}. TargetLength: {TargetLength}";
    }
}
