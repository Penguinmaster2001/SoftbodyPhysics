
namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct Spring
{
    public int StartVertex;
    public int EndVertex;
    public float LengthAreaCoefficient;
    public float TargetLength;



    public override readonly string ToString()
    {
        return $"Spring from {StartVertex} to {EndVertex}. LengthAreaCoefficient: {LengthAreaCoefficient}. TargetLength: {TargetLength}";
    }
}
