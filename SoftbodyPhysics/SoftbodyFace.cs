
using System;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public struct SoftbodyFace
{
    public int V0;
    public int V1;
    public int V2;

    public int this[int vertex]
    {
        readonly get => vertex % 3 switch
            {
                0 => V0,
                1 => V1,
                2 => V2,
                _ => throw new ArgumentOutOfRangeException(nameof(vertex), "Index out of range")
            };
        set
        {
            switch (vertex % 3)
            {
                case 0: V0 = value; break;
                case 1: V1 = value; break;
                case 2: V2 = value; break;
                default: throw new ArgumentOutOfRangeException(nameof(vertex), "Index out of range");
            }
        }
    }
}
