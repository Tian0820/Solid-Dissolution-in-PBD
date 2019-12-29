using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;

using PositionBasedDynamics.Collisions;
using PositionBasedDynamics.Constraints;
using PositionBasedDynamics.Sources;

namespace PositionBasedDynamics.Bodies
{

    public class Body3d
    {

        public double Stiffness { get; private set; }

        public double Viscosity { get; set; }

        public double[] Lambda { get; private set; }

        internal CubicKernel3d Kernel { get; private set; }

        internal ParticleHash3d FluidHash { get; private set; }

        internal ParticleHash3d RigidHash { get; private set; }

        public int NumParticles { get { return Particles.Count; } }

        public int NumConstraints { get { return Constraints.Count; } }

        public double Dampning { get; set; }

        public List<Particle> Particles { get; set; }

        public Box3d Bounds { get; private set; }

        public List<Constraint3d> Constraints { get; set; }

        private List<StaticConstraint3d> StaticConstraints { get; set; }

        public Body3d(int numParticles, double radius, double mass)
        {
            InitBody3d(numParticles, radius, mass);
        }

        // particleAttr: density if FLUID, mass if RIGID
        public Body3d(ParticlePhase phase, ParticleSource source, double radius, double particleAttr, Matrix4x4d RTS)
        {
            InitBody3d(source.NumParticles, radius, 1.0);
            if (phase == ParticlePhase.FLUID)
            {
                for (int i = 0; i < source.NumParticles; i++)
                {
                    double d = radius * 2;
                    Particle newParticle = new Particle(radius, 0.8 * d * d * d * particleAttr, particleAttr, ParticlePhase.FLUID);
                    Particles.Add(newParticle);
                }
                Viscosity = 0.02;
                Dampning = 0;


                CreateParticles(source, RTS);
                InitFluidAttr();
            } else if (phase == ParticlePhase.RIGID)
            {
                for (int i = 0; i < source.NumParticles; i++)
                {
                    double mass = particleAttr;
                    Particle newParticle = new Particle(radius, mass, 0, ParticlePhase.RIGID);
                    Particles.Add(newParticle);
                }

                Stiffness = 1.0;

                CreateParticles(source, RTS);
                Constraints.Add(new ShapeMatchingConstraint3d(this, particleAttr, Stiffness));
            }
        }

        private void InitBody3d(int numParticles, double radius, double mass)
        {
            Particles = new List<Particle>();
            Constraints = new List<Constraint3d>();
            StaticConstraints = new List<StaticConstraint3d>();
            Dampning = 1;
        }

        private void InitFluidAttr()
        {
            double cellSize = Particles[0].ParticleRadius * 4.0;
            Kernel = new CubicKernel3d(cellSize);

            FluidHash = new ParticleHash3d(NumParticles, cellSize);

            Lambda = new double[NumParticles];
        }

        public Body3d ContactBody3d(Body3d body)
        {
            Body3d newBody = this;

            newBody.Particles.AddRange(body.Particles);

            newBody.Constraints.AddRange(body.Constraints);
            newBody.StaticConstraints.AddRange(body.StaticConstraints);
            newBody.FluidHash = new ParticleHash3d(newBody.NumParticles, newBody.Particles[0].ParticleRadius * 4.0);
            newBody.Lambda = new double[newBody.NumParticles];

            return newBody;
        }

        internal void ConstrainPositions(double di)
        {
            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].ConstrainPositions(di);
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainPositions(di);
            }
        }

        internal void ConstrainVelocities()
        {

            for (int i = 0; i < Constraints.Count; i++)
            {
                Constraints[i].ConstrainVelocities();
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainVelocities();
            }

        }

        public void RandomizePositions(System.Random rnd, double amount)
        {
            for(int i = 0; i < NumParticles; i++)
            {
                double rx = rnd.NextDouble() * 2.0 - 1.0;
                double ry = rnd.NextDouble() * 2.0 - 1.0;
                double rz = rnd.NextDouble() * 2.0 - 1.0;

                Particles[i].Position += new Vector3d(rx, ry, rz) * amount;
            }
        }

        public void RandomizeConstraintOrder(System.Random rnd)
        {
            int count = Constraints.Count;
            if (count <= 1) return;

            List<Constraint3d> tmp = new List<Constraint3d>();

            while (tmp.Count != count)
            {
                int i = rnd.Next(0, Constraints.Count - 1);

                tmp.Add(Constraints[i]);
                Constraints.RemoveAt(i);
            }

            Constraints = tmp;
        }

        public void MarkAsStatic(Box3d bounds)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                if (bounds.Contains(Particles[i].Position))
                {
                    StaticConstraints.Add(new StaticConstraint3d(this, i));
                }
            }
        }

        public void UpdateBounds()
        {
            Vector3d min = new Vector3d(double.PositiveInfinity);
            Vector3d max = new Vector3d(double.NegativeInfinity);

            List<Vector3d> postions = new List<Vector3d>();
            for(int i = 0; i < Particles.Count; i++)
            {
                postions.Add(Particles[i].Position);
            }

            for (int i = 0; i < NumParticles; i++)
            {
                min.Min(postions[i]);
                max.Max(postions[i]);
            }

            double radius = Particles[0].ParticleRadius;
            min -= radius;
            max += radius;

            Bounds = new Box3d(min, max);
        }

        public void AddBoundry(FluidBoundary3d boundry)
        {
            FluidConstraint3d constraint = new FluidConstraint3d(this, boundry);

            Constraints.Add(constraint);
        }

        internal void Reset()
        {
            for (int i = 0; i < NumParticles; i++)
            {
                Lambda[i] = 0.0;
                Particles[i].DynamicDensity = 0.0;
                //Densities[i] = 0.0;
            }
        }

        public void RandomizePositionOrder(System.Random rnd)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                Vector3d tmp = Particles[i].Position;

                int idx = rnd.Next(0, NumParticles - 1);
                Particles[i].Position = Particles[idx].Position;
                Particles[idx].Position = tmp;
            }
        }

        internal void ComputeViscosity()
        {
            int[,] neighbors = new int[Particles.Count, FluidHash.MaxNeighbors];
            int[] numNeighbors = new int[Particles.Count];

            for (int i = 0; i < Particles.Count; i++)
            {
                numNeighbors[i] = Particles[i].NeighbourIndexes.Count;
                for (int j = 0; j < numNeighbors[i]; j++)
                {
                    neighbors[i, j] = Particles[i].NeighbourIndexes[j];
                }
            }

            double viscosityMulMass = Viscosity * Particles[0].ParticleMass;

            // Compute viscosity forces (XSPH) 
            for (int i = 0; i < NumParticles; i++)
            {
                //Viscosity for particle Pi. Modifies the velocity.
                //Vi = Vi + c * SUMj Vij * W(Pi - Pj, h)
                Vector3d pi = Particles[i].Predicted;

                for (int j = 0; j < numNeighbors[i]; j++)
                {
                    int neighborIndex = neighbors[i, j];
                    if (neighborIndex < NumParticles) // Test if fluid particle
                    {
                        double invDensity = 1.0 / Particles[neighborIndex].DynamicDensity;
                        Vector3d pn = Particles[neighborIndex].Predicted;

                        double k = Kernel.W(pi.x - pn.x, pi.y - pn.y, pi.z - pn.z) * viscosityMulMass * invDensity;
                        Particles[i].Velocity -= k * (Particles[i].Velocity - Particles[neighborIndex].Velocity);
                    }
                }
            }

        }

        private void CreateParticles(ParticleSource source, Matrix4x4d RTS)
        {

            for (int i = 0; i < NumParticles; i++)
            {
                Vector4d pos = RTS * source.Positions[i].xyz1;
                Particles[i].Position = new Vector3d(pos.x, pos.y, pos.z);
                Particles[i].Predicted = Particles[i].Position;
            }

        }
    }

}
