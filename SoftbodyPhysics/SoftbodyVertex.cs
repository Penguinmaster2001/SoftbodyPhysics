
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyVertex : IVertex
{
    public Vector3 Position { get; set; }
    public Vector3 Velocity;
    public Vector3 Force;
    public float Mass;

    public Vector2 UV { get; set; }
    public Vector3 Normal { get; set; }



    public override readonly string ToString()
    {
        return $"SoftbodyVertex at {Position}\tVelocity: {Velocity}\tForce: {Force}\tMass: {Mass}";
    }
}
