using System;
using Common.Mathematics.LinearAlgebra;
using UnityEngine;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{
    internal class FluidClothContact3d : CollisionContact3d
    {
        private Body3d FluidBody, ClothBody;

        private int i0, i1;

        private double Diameter, Diameter2;

        private double Mass0, Mass1;

        internal FluidClothContact3d(Body3d clothbody, int i0, Body3d fluidBody, int i1)
        {
            FluidBody = fluidBody;
            this.i1 = i1;

            ClothBody = clothbody;
            this.i0 = i0;

            Diameter = FluidBody.Particles[0].ParticleRadius + ClothBody.Particles[0].ParticleRadius;
            Diameter2 = Diameter * Diameter;

            double sum = FluidBody.Particles[0].ParticleMass + ClothBody.Particles[0].ParticleMass;
            Mass0 = ClothBody.Particles[0].ParticleMass / sum;
            Mass1 = FluidBody.Particles[0].ParticleMass / sum;
            //Debug.Log("Mass0: " + Mass0);
            //Debug.Log("Mass1: " + Mass1);
        }

        internal override void ResolveContact(double di)
        {
            //Debug.Log("ResolveContact cloth fluid!");
            Vector3d normal = FluidBody.Particles[i1].Predicted - ClothBody.Particles[i0].Predicted;

            double sqLen = normal.SqrMagnitude;

            if (sqLen <= Diameter2 && sqLen > 1e-9)
            {
                double len = Math.Sqrt(sqLen);
                normal /= len;

                Vector3d delta = di * (Diameter - len) * normal; 

                FluidBody.Particles[i1].Predicted += 2 * delta * Mass1;
                FluidBody.Particles[i1].Position += 2 * delta * Mass1;

                ClothBody.Particles[i0].Predicted -= 2 * delta * Mass1;
                ClothBody.Particles[i0].Position -= 2 * delta * Mass1;

                if (!ClothBody.Particles[i0].AbsorbedIndexes.Contains(i1))
                {
                    ClothBody.Particles[i0].AbsorbedIndexes.Add(i1);
                    ClothBody.Particles[i0].AbsorbPhase = true;
                }
                else
                {
                    ClothBody.Particles[i0].AbsorbPhase = false;
                }
            }
        }

    }
}
