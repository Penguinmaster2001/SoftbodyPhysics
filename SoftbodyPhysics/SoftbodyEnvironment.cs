
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public partial class SoftbodyEnvironment: Node3D, ISoftbodyEnvironment
{
    [Export]
    public Vector3 Gravity { get; set; }

    [Export]
    public float AirTemperature { get; set; }

    [Export]
    public float AirDensity { get; set; }

    [Export]
    public float AirPressure { get; set; }
}
