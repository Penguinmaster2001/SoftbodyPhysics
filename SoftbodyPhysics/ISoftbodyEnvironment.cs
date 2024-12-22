
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public interface ISoftbodyEnvironment
{
    Vector3 Gravity { get; }
    float AirTemperature { get; }
    float AirDensity { get; }
    float AirPressure { get; }
}
