
namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyGradient
{
    public float ThermalFlux;

    public SoftbodyGradientVertex[] Vertices;



    public SoftbodyGradient(int numVertices)
    {
        ThermalFlux = 0.0f;
        Vertices = new SoftbodyGradientVertex[numVertices];
    }



    public static SoftbodyGradient operator *(float scalar, SoftbodyGradient gradient)
    {
        SoftbodyGradient scaledGradient = new() {
            ThermalFlux = scalar * gradient.ThermalFlux,
            Vertices = new SoftbodyGradientVertex[gradient.Vertices.Length]
        };

        for (int vert = 0; vert < gradient.Vertices.Length; vert++)
        {
            scaledGradient.Vertices[vert] = new() {
                Force = scalar * gradient.Vertices[vert].Force
            };
        }

        return scaledGradient;
    }



    public static SoftbodyGradient operator +(SoftbodyGradient gradientA, SoftbodyGradient gradientB)
    {
        SoftbodyGradient gradientSum = new() {
            ThermalFlux = gradientA.ThermalFlux + gradientB.ThermalFlux,
            Vertices = new SoftbodyGradientVertex[gradientA.Vertices.Length]
        };

        for (int vert = 0; vert < gradientA.Vertices.Length; vert++)
        {
            gradientSum.Vertices[vert] = new() {
                Force = gradientA.Vertices[vert].Force + gradientB.Vertices[vert].Force
            };
        }

        return gradientSum;
    }
}
