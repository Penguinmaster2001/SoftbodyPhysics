
using Godot;

using System;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public class SoftbodyState
{
    public SoftbodyVertex[] Vertices;
    public Spring[] Springs;

    public float Temperature;
    public float ThermalFlux;
    public float Pressure;
    public float Volume;
    public float SurfaceArea;



    public static SoftbodyState operator *(float dt, SoftbodyState state)
    {
        throw new NotImplementedException();
    }



    public static SoftbodyState operator +(SoftbodyState stateA, SoftbodyState stateB)
    {
        throw new NotImplementedException();
    }
}
