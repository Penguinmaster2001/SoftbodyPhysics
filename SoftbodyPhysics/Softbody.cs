
using System;
using Godot;



namespace GodotSoftbodyPhysics.SoftbodyPhysics;



public class Softbody
{
    #region State
    public SoftbodyState State;
    public Spring[] Springs;
    public SoftbodyFace[] Faces;
    public int[] Indices;
    #endregion


    #region Initial Conditions
    public float StartVolume;
    #endregion


    #region System Variables
    public SoftbodyEnvironment Environment;
    public float _youngsModulus;
    public float _preCompress;
    public float _initialSkinAreaMass;
    public float _damping;
    public float _pressureConstant;
    public float _mols;
    public float _skinThermalConductivity;
    public float _gasMolDensity;
    public float _dragMultiplier;
    #endregion


    #region Simulation Settings
    private const float _epsilon = 0.001f;
    private const float _epsilonSquared = 0.0001f;
    public float MaxForce;
    public int SubSteps;
    public double TimeSpeed;
    #endregion



    public void Update(double dt)
    {
        float subStepDt = (float)(dt / SubSteps);

        for (int subStep = 0; subStep < SubSteps; subStep++)
        {
            UpdateRk4(subStepDt);
        }
    }



    private void UpdateEuler(float dt)
    {
        SoftbodyGradient k1 = CalculateGradient(State);
        State = TimeStepGradient(State, k1, dt);
    }



    private void UpdateRk4(float dt)
    {
        SoftbodyGradient k1 = CalculateGradient(State);
        SoftbodyGradient k2 = CalculateGradient(TimeStepGradient(State, k1, 0.5f * dt));
        SoftbodyGradient k3 = CalculateGradient(TimeStepGradient(State, k2, 0.5f * dt));
        SoftbodyGradient k4 = CalculateGradient(TimeStepGradient(State, k3, dt));

        State = TimeStepGradient(State, k1 + (2.0f * k2) + (2.0f * k3) + k4, 0.166667f * dt);
    }



    private SoftbodyGradient CalculateGradient(SoftbodyState state)
    {
        SoftbodyGradient gradient = new(state.Vertices.Length);
        CalculateSprings(ref state, ref gradient);
        CalculateInternalPressure(ref state, ref gradient);
        CalculateEnvironment(ref state, ref gradient);
        
        return gradient;
    }



    public SoftbodyState TimeStepGradient(SoftbodyState state, SoftbodyGradient gradient, float dt)
    {
        SoftbodyState newState = state;
        newState.Temperature += dt * gradient.ThermalFlux;

        for (int vert = 0; vert < newState.Vertices.Length; vert++)
        {
            Vector3 dv = gradient.Vertices[vert].Force * (dt / (newState.Vertices[vert].InitialArea * _initialSkinAreaMass));
            newState.Vertices[vert].Position += (newState.Vertices[vert].Velocity + dv * 0.5f) * dt;
            newState.Vertices[vert].Velocity += dv;
        }

        return newState;
    }



    public float CalculateVolume()
    {
        float totalVolume = 0.0f;
        for (int faceIndex = 0; faceIndex < Faces.Length; faceIndex++)
        {
            SoftbodyFace face = Faces[faceIndex];
            Vector3 v0 = State.Vertices[face.V0].Position;
            Vector3 v1 = State.Vertices[face.V1].Position;
            Vector3 v2 = State.Vertices[face.V2].Position;
            totalVolume += v2.Dot(v1.Cross(v0));
        }

        return MathF.Max(0.16667f * totalVolume, _epsilon);
    }



    private void CalculateEnvironment(ref SoftbodyState state, ref SoftbodyGradient gradient)
    {
        state.GasMass = _mols * _gasMolDensity;

        for (int vert = 0; vert < state.Vertices.Length; vert++)
        {
            // Gravity per vertex
            gradient.Vertices[vert].Force += ((state.Vertices[vert].InitialArea * _initialSkinAreaMass) + state.GasMass / state.Vertices.Length) * Environment.Gravity;

            // Buoyancy per vertex
            gradient.Vertices[vert].Force += Environment.AirDensity * state.Volume / state.Vertices.Length * -Environment.Gravity;

            if (state.Vertices[vert].Position.Y < 0.0f)
            {
                gradient.Vertices[vert].Force *= 0.5f;
                gradient.Vertices[vert].Force += 1.5f * gradient.Vertices[vert].Force.Y * state.Vertices[vert].InitialArea * state.Vertices[vert].Position.Y * Vector3.Up;
                state.Vertices[vert].Position = state.Vertices[vert].Position * new Vector3(1.0f, 0.0f, 1.0f);
            }
        }
    }




    private void CalculateSprings(ref SoftbodyState state, ref SoftbodyGradient gradient)
    {
        for (int springIndex = 0; springIndex < Springs.Length; springIndex++)
        {
            Spring spring = Springs[springIndex];


            SoftbodyVertex startVertex = state.Vertices[spring.StartVertex];
            SoftbodyVertex endVertex = state.Vertices[spring.EndVertex];
            
            Vector3 springVector = startVertex.Position - endVertex.Position;
            float springLength = springVector.Length();
            if (springLength < _epsilon)
            {
                springLength = _epsilon;
            }

            Vector3 springDirection = springVector / springLength;
            Vector3 hookesFactor = spring.LengthAreaCoefficient * _youngsModulus * (spring.TargetLength - (springLength * _preCompress)) * springDirection;

            Vector3 startVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -startVertex.Velocity.Project(springDirection);
            Vector3 endVertDampingVel = springDirection.LengthSquared() < _epsilonSquared ? Vector3.Zero : -endVertex.Velocity.Project(springDirection);

            gradient.Vertices[spring.StartVertex].Force += LimitInfiniteLength(hookesFactor + spring.Damping * startVertDampingVel, MaxForce);
            gradient.Vertices[spring.EndVertex].Force += LimitInfiniteLength(-hookesFactor + spring.Damping * endVertDampingVel, MaxForce);
        }
    }



    private void CalculateInternalPressure(ref SoftbodyState state, ref SoftbodyGradient gradient)
    {
        float newVolume = CalculateVolume();
        state.Pressure = _mols * _pressureConstant * state.Temperature / newVolume;

        state.SurfaceArea = 0.0f;
        
        for (int faceIndex = 0; faceIndex < Faces.Length; faceIndex++)
        {
            SoftbodyFace face = Faces[faceIndex];
            Vector3 v0 = state.Vertices[face.V0].Position;
            Vector3 v1 = state.Vertices[face.V1].Position;
            Vector3 v2 = state.Vertices[face.V2].Position;

            Vector3 aveVel = 0.3333f * 
                            (state.Vertices[face.V0].Velocity +
                             state.Vertices[face.V1].Velocity +
                             state.Vertices[face.V2].Velocity);
            
            // The length of this is twice the area of the tri
            // The direction is the normal of the tri
            Vector3 cross = (v2 - v0).Cross(v1 - v0);
            float area = 0.5f * cross.Length();
            state.SurfaceArea += area;

            Vector3 dragForce = aveVel * (_dragMultiplier * Environment.AirDensity * area * -aveVel.Length());

            Vector3 pressureForce = cross * ((0.5f * state.Pressure) - Environment.AirPressure);
            Vector3 forcePerVertex = LimitInfiniteLength(0.3333f * (pressureForce + dragForce), MaxForce);
            gradient.Vertices[face.V0].Force += forcePerVertex;
            gradient.Vertices[face.V1].Force += forcePerVertex;
            gradient.Vertices[face.V2].Force += forcePerVertex;
        }


        gradient.ThermalFlux = Mathf.Clamp(_skinThermalConductivity * state.SurfaceArea * (Environment.AirTemperature - state.Temperature), -MaxForce, MaxForce);
        state.Temperature = Mathf.Clamp(state.Temperature + (0.66667f * (-state.Pressure * (newVolume - state.Volume)) / (_mols * _pressureConstant)), 0.0f, MaxForce);
        state.Volume = newVolume;
    }



    private static Vector3 LimitInfiniteLength(Vector3 vector, float length)
    {
        if (!vector.IsFinite())
        {
            vector = new() {
                X = float.IsNegativeInfinity(vector.X) ? -1.0f : (float.IsPositiveInfinity(vector.X) ? 1.0f : 0.0f),
                Y = float.IsNegativeInfinity(vector.Y) ? -1.0f : (float.IsPositiveInfinity(vector.Y) ? 1.0f : 0.0f),
                Z = float.IsNegativeInfinity(vector.Z) ? -1.0f : (float.IsPositiveInfinity(vector.Z) ? 1.0f : 0.0f)
            };
        }

        return vector.LimitLength(length);
    }
}
