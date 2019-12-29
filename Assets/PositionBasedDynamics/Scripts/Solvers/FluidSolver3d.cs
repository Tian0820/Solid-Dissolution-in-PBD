using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Geometry.Shapes;
using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Solvers
{

    public class FluidSolver3d
    {

        private Body3d Body { get; set; }

        private List<ExternalForce3d> Forces { get; set; }

        public FluidSolver3d(Body3d body)
        {
            Body = body;
            Forces = new List<ExternalForce3d>();
        }

        public void AddForce(ExternalForce3d force)
        {
            Forces.Add(force);
        }

        public void StepPhysics(double dt)
        {

            if (dt == 0.0) return;
            
            AppyExternalForces(dt);

            EstimatePositions(dt);

            UpdateConstraint();

            UpdateVelocities(dt);

            Body.ComputeViscosity();

            UpdatePositions();
        }

        private void AppyExternalForces(double dt)
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


    }

}