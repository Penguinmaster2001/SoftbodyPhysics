
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyVertex : IVertex
{
    public Vector3 Position { get; set; }
    public Vector3 Velocity;
    public float InitialArea;

    public Vector2 UV { get; set; }
    public Vector3 Normal { get; set; }



    public override readonly string ToString()
    {
        return $"SoftbodyVertex at {Position}\tVelocity: {Velocity}\tMass: {InitialArea}";
    }
}
