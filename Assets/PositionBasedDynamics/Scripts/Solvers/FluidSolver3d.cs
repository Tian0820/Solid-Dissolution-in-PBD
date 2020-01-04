using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Geometry.Shapes;
using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Collisions;

namespace PositionBasedDynamics.Solvers
{

    public class FluidSolver3d
    {

        public Body3d Body { get; set; }

        public List<Particle> ParticleToTrans { get; set; }

        private List<ExternalForce3d> Forces { get; set; }

        public int DissolutionRate { get; set; }

        private int IterNum;

        public FluidSolver3d(Body3d body)
        {
            Body = body;
            Forces = new List<ExternalForce3d>();
            ParticleToTrans = new List<Particle>();
            IterNum = 1;
        }

        public void UpdateParticleToTrans(List<Particle> particles)
        {
            ParticleToTrans = particles;
            Body.ContactParticle(ParticleToTrans);
        }

        public void AddForce(ExternalForce3d force)
        {
            Forces.Add(force);
        }

        public void StepPhysics(double dt)
        {

            if (dt == 0.0) return;
            
            ApplyExternalForces(dt);

            EstimatePositions(dt);

            UpdateConstraint();

            UpdateVelocities(dt);

            Body.ComputeViscosity();

            UpdatePositions();

            //if (IterNum % DissolutionRate == 0)
                UpdateColor();

            IterNum++;
        }

        private void ApplyExternalForces(double dt)
        {
            for (int i = 0; i < Body.NumParticles; i++)
            {
                Body.Particles[i].Velocity -= (Body.Particles[i].Velocity * Body.Dampning) * dt;
            }

            for (int i = 0; i < Forces.Count; i++)
            {
                Forces[i].ApplyForce(dt, Body);
            }
        }

        private void EstimatePositions(double dt)
        {
            for (int i = 0; i < Body.NumParticles; i++)
            {
                Body.Particles[i].Predicted = Body.Particles[i].Position + dt * Body.Particles[i].Velocity;
                //Debug.Log("EstimatePositions: " + Body.Particles[i].Velocity);
            }
        }

        private void UpdateConstraint()
        {
            Body.ConstrainPositions(1);
        }

        private void UpdateVelocities(double dt)
        {
            double inv_dt = 1.0 / dt;

            for (int i = 0; i < Body.NumParticles; i++)
            {
                Vector3d d = Body.Particles[i].Predicted - Body.Particles[i].Position;
                Body.Particles[i].Velocity = d * inv_dt;
            }
        }

        private void UpdatePositions()
        {
            for (int i = 0; i < Body.NumParticles; i++)
            {
                Body.Particles[i].Position = Body.Particles[i].Predicted;
            }
        }

        private void UpdateColor()
        {
            Vector4d color = new Vector4d(0, 0, 0, 0);
            for (int i = 0; i < Body.NumParticles; i++)
            {
                color = new Vector4d(0, 0, 0, 0);
                int fluidNeighbourNum = 0;
                for (int j = 0; j < Body.Particles[i].NeighbourIndexes.Count; j++)
                {
                    int neighborIndex = Body.Particles[i].NeighbourIndexes[j];
                    if (neighborIndex < Body.NumParticles)
                    {
                        if (fluidNeighbourNum == 20)
                            break;
                        color += Body.Particles[neighborIndex].Color;
                        fluidNeighbourNum++;
                    }
                }
                color /= fluidNeighbourNum;
                if (fluidNeighbourNum > 0)
                {
                    Body.Particles[i].Color = color;
                }
            }
        }


    }

}