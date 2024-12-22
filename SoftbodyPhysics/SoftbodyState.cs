
namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyState
{
    public float Temperature;
    public float Pressure;
    public float Volume;
    public float SurfaceArea;
    public float GasMass;

    public SoftbodyVertex[] Vertices;


    public SoftbodyState(SoftbodyState stateToCopy)
    {
        Temperature = stateToCopy.Temperature;
        Pressure = stateToCopy.Pressure;
        Volume = stateToCopy.Volume;
        SurfaceArea = stateToCopy.SurfaceArea;
        GasMass = stateToCopy.GasMass;
        Vertices = stateToCopy.Vertices;
    }
}
