
using Godot;



namespace GodotSoftbodyPhysics;



public interface IVertex
{
    Vector3 Position { get; set; }
    Vector2 UV { get; set; }
    Vector3 Normal { get; set; }
}
