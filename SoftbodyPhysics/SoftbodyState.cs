
using Godot;

using System;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyState
{
    public float Temperature;
    public float ThermalFlux;
    public float Pressure;
    public float Volume;
    public float SurfaceArea;
    public float GasMass;

    public SoftbodyVertex[] Vertices;


    public SoftbodyState(SoftbodyState stateToCopy)
    {
        Temperature = stateToCopy.Temperature;
        ThermalFlux = stateToCopy.ThermalFlux;
        Pressure = stateToCopy.Pressure;
        Volume = stateToCopy.Volume;
        SurfaceArea = stateToCopy.SurfaceArea;
        GasMass = stateToCopy.GasMass;
        Vertices = stateToCopy.Vertices;
    }



    public static SoftbodyState operator *(float dt, SoftbodyState state)
    {
        SoftbodyState newState = new()
        {
            Temperature = dt * state.ThermalFlux,
            ThermalFlux = state.ThermalFlux,
            Pressure = state.Pressure,
            Volume = state.Volume,
            SurfaceArea = state.SurfaceArea,
            GasMass = state.GasMass,
            Vertices = new SoftbodyVertex[state.Vertices.Length]
        };

        for (int vert = 0; vert < state.Vertices.Length; vert++)
        {
            Vector3 dv = state.Vertices[vert].Force * (dt / state.Vertices[vert].Mass);
            newState.Vertices[vert] = new() {
                Position = (state.Vertices[vert].Velocity + dv * 0.5f) * dt,
                Velocity = dv,
            };
        }

        return newState;
    }



    public static SoftbodyState operator +(SoftbodyState stateA, SoftbodyState stateB)
    {
        SoftbodyState newState = new()
        {
            Temperature = stateA.Temperature + stateB.Temperature,
            ThermalFlux = stateA.ThermalFlux,
            Pressure = stateA.Pressure,
            Volume = stateA.Volume,
            SurfaceArea = stateA.SurfaceArea,
            Vertices = new SoftbodyVertex[stateA.Vertices.Length]
        };

        for (int vert = 0; vert < stateA.Vertices.Length; vert++)
        {
            newState.Vertices[vert] = new() {
                Position = stateA.Vertices[vert].Position + stateB.Vertices[vert].Position,
                Velocity = stateA.Vertices[vert].Velocity + stateB.Vertices[vert].Velocity,
                Mass = stateA.Vertices[vert].Mass,
                UV = stateA.Vertices[vert].UV,
                Normal = stateA.Vertices[vert].Normal
            };
        }

        return newState;
    }
}
