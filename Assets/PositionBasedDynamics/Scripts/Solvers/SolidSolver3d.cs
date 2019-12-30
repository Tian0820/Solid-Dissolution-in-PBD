using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Collisions;

namespace PositionBasedDynamics.Solvers
{

    public class SolidSolver3d
    {
        public int SolverIterations { get; set; }

        public int CollisionIterations { get; set; }

        public double SleepThreshold { get; set; }

        public List<Body3d> SolidBodies { get; private set; }

        public Body3d FluidBody { get; set; }

        private List<ExternalForce3d> Forces { get; set; }

        private List<Collision3d> Collisions { get; set; }

        private bool FluidSolidContactActive = true;

        public SolidSolver3d()
        {
            SolverIterations = 4;
            CollisionIterations = 2;

            Forces = new List<ExternalForce3d>();
            Collisions = new List<Collision3d>();
            SolidBodies = new List<Body3d>();
        }

        public void AddForce(ExternalForce3d force)
        {
            if (Forces.Contains(force)) return;
            Forces.Add(force);
        }

        public void AddCollision(Collision3d collision)
        {
            if (Collisions.Contains(collision)) return;
            Collisions.Add(collision);
        }

        public void AddBody(Body3d body)
        {
            if (SolidBodies.Contains(body)) return;
            SolidBodies.Add(body);
        }

        public void StepPhysics(double dt)
        {
            if (dt == 0.0) return;

            AppyExternalForces(dt);

            EstimatePositions(dt);

            UpdateBounds();

            ResolveCollisions();

            ConstrainPositions();

            UpdateVelocities(dt);

            UpdatePositions();

            UpdateBounds();

        }

        private void AppyExternalForces(double dt)
        {

            for (int j = 0; j < SolidBodies.Count; j++)
            {
                Body3d body = SolidBodies[j];

                for (int i = 0; i < body.NumParticles; i++)
                {
                    body.Particles[i].Velocity -= (body.Particles[i].Velocity * body.Dampning) * dt;
                }

                for (int i = 0; i < Forces.Count; i++)
                {
                    Forces[i].ApplyForce(dt, body);
                }
            }
        }

        private void EstimatePositions(double dt)
        {
            for (int j = 0; j < SolidBodies.Count; j++)
            {
                Body3d body = SolidBodies[j];

                for (int i = 0; i < body.NumParticles; i++)
                {
                    body.Particles[i].Predicted = body.Particles[i].Position + dt * body.Particles[i].Velocity;
                }
            }
        }

        private void UpdateBounds()
        {
            for (int i = 0; i < SolidBodies.Count; i++)
            {
                SolidBodies[i].UpdateBounds();
            }
        }

        private void ResolveCollisions()
        {
            List<CollisionContact3d> contacts = new List<CollisionContact3d>();

            Collision3d fluidSolidCollision = Collisions[0];
            Collision3d PlanarCollision = Collisions[1];

            //for (int j = 0; j < SolidBodies.Count && FluidSolidContactActive; j++)
            for (int j = 0; j < SolidBodies.Count; j++)
            {
                fluidSolidCollision.FindContacts(SolidBodies[j], FluidBody, contacts);
            }

            FluidSolidContactActive = !PlanarCollision.FindContacts(SolidBodies, contacts);

            double di = 1.0 / CollisionIterations;

            for(int i = 0; i < CollisionIterations; i++)
            {
                for (int j = 0; j < contacts.Count; j++)
                {
                    contacts[j].ResolveContact(di);
                }
            }
        }

        private void ConstrainPositions()
        {
            double di = 1.0 / SolverIterations;

            for (int i = 0; i < SolverIterations; i++)
            {
                for (int j = 0; j < SolidBodies.Count; j++)
                {
                    SolidBodies[j].ConstrainPositions(di);
                }
            }
        }

        private void UpdateVelocities(double dt)
        {
            double invDt = 1.0 / dt;
            double threshold2 = SleepThreshold * dt;
            threshold2 *= threshold2;

            for (int j = 0; j < SolidBodies.Count; j++)
            {
                Body3d body = SolidBodies[j];

                for (int i = 0; i < body.NumParticles; i++)
                {
                    Vector3d d = body.Particles[i].Predicted - body.Particles[i].Position;
                    body.Particles[i].Velocity = d * invDt;

                    double m = body.Particles[i].Velocity.SqrMagnitude;
                    if (m < threshold2)
                        body.Particles[i].Velocity = Vector3d.Zero;
                }
            }
        }

        private void ConstrainVelocities()
        {
            for (int i = 0; i < SolidBodies.Count; i++)
            {
                SolidBodies[i].ConstrainVelocities();
            }
        }

        private void UpdatePositions()
        {
            for (int j = 0; j < SolidBodies.Count; j++)
            {
                Body3d body = SolidBodies[j];

                for (int i = 0; i < body.NumParticles; i++)
                {
                    body.Particles[i].Position = body.Particles[i].Predicted;
                }
            }
        }

    }

}