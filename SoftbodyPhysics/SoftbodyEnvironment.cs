
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class SoftbodyEnvironment: Node3D
{
    [Export]
    public Vector3 Gravity;

    [Export]
    public float AirTemperature;

    [Export]
    public float AirDensity;
}
