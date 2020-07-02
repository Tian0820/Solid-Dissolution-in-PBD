using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Collisions;
using PositionBasedDynamics.Utilities;

namespace PositionBasedDynamics.Solvers
{

    public class ClothSolver3d
    {
        public int SolverIterations { get; set; }

        public int CollisionIterations { get; set; }

        public double SleepThreshold { get; set; }

        public List<Body3d> ClothBodies { get; private set; }

        public Body3d FluidBody { get; set; }

        public int DissolutionRate { get; set; }

        public List<Particle> ParticleToTrans { get; set; }

        private List<ExternalForce3d> Forces { get; set; }

        private List<Collision3d> Collisions { get; set; }

        private Util Util;

        private int IterNum;

        public ClothSolver3d()
        {
            SolverIterations = 4;
            CollisionIterations = 2;
            IterNum = 1;

            ParticleToTrans = new List<Particle>();
            Forces = new List<ExternalForce3d>();
            Collisions = new List<Collision3d>();
            ClothBodies = new List<Body3d>();
            Util = new Util();
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
            if (ClothBodies.Contains(body)) return;
            ClothBodies.Add(body);
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

            AbsorbWater();

            UpdatePositions();

            UpdateBounds();

            IterNum++;

        }

        private void AppyExternalForces(double dt)
        {

            for (int j = 0; j < ClothBodies.Count; j++)
            {
                Body3d body = ClothBodies[j];

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
            for (int j = 0; j < ClothBodies.Count; j++)
            {
                Body3d body = ClothBodies[j];

                for (int i = 0; i < body.NumParticles; i++)
                {
                    body.Particles[i].Predicted = body.Particles[i].Position + dt * body.Particles[i].Velocity;
                }
            }
        }

        private void UpdateBounds()
        {
            for (int i = 0; i < ClothBodies.Count; i++)
            {
                ClothBodies[i].UpdateBounds();
            }
        }

        private void ResolveCollisions()
        {
            List<CollisionContact3d> contacts = new List<CollisionContact3d>();

            Collision3d fluidClothCollision = Collisions[0];
            Collision3d PlanarCollision = Collisions[1];

            PlanarCollision.FindContacts(ClothBodies, contacts);

            for (int i = 0; i < ClothBodies.Count; i++)
            {
                fluidClothCollision.FindContacts(ClothBodies[i], FluidBody, contacts);
            }

            double di = 1.0 / CollisionIterations;

            for (int i = 0; i < CollisionIterations; i++)
            {
                for (int j = 0; j < contacts.Count; j++)
                {
                    contacts[j].ResolveContact(di);
                }
            }
        }

        private void AbsorbWater()
        {
            // compute saturation
            foreach (ClothBody3d body in ClothBodies)
                body.computeSaturation();
            // absorb water
            foreach (ClothBody3d body in ClothBodies)
            {
                int numParticles = body.Particles.Count;
                for (int i = 0; i < numParticles; i++)
                {
                    //if (body.Particles[i].AbsorbedIndexes.Count > 0)
                    //    Debug.Log("Absorption " + i + ": " + body.Particles[i].AbsorbedIndexes.Count + "; " + body.Saturations[i]);
                    if (body.Particles[i].AbsorbPhase)
                    {
                        Vector4d newColor = Util.ChangeColor(body.Particles[i].Color, body.Saturations[i]);
                        body.Particles[i].Color = newColor;
                        //Debug.Log("absorb phase");
                    }
                    //else
                    //{

                    //}
                }
            }
            //Debug.Log("===============");
        }

        private void ConstrainPositions()
        {
            double di = 1.0 / SolverIterations;

            for (int i = 0; i < SolverIterations; i++)
            {
                for (int j = 0; j < ClothBodies.Count; j++)
                {
                    ClothBodies[j].ConstrainPositions(di);
                }
            }
        }

        private void UpdateVelocities(double dt)
        {
            double invDt = 1.0 / dt;
            double threshold2 = SleepThreshold * dt;
            threshold2 *= threshold2;

            for (int j = 0; j < ClothBodies.Count; j++)
            {
                Body3d body = ClothBodies[j];

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
            for (int i = 0; i < ClothBodies.Count; i++)
            {
                ClothBodies[i].ConstrainVelocities();
            }
        }

        private void UpdatePositions()
        {
            for (int j = 0; j < ClothBodies.Count; j++)
            {
                Body3d body = ClothBodies[j];

                for (int i = 0; i < body.NumParticles; i++)
                {
                    body.Particles[i].Position = body.Particles[i].Predicted;
                }
            }
        }

    }

}
