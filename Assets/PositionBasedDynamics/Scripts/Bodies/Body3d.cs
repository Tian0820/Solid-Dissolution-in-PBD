using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;

using PositionBasedDynamics.Collisions;
using PositionBasedDynamics.Constraints;
using PositionBasedDynamics.Sources;
using PositionBasedDynamics.Bodies.Fluids;

namespace PositionBasedDynamics.Bodies
{

    public class Body3d
    {
        //public List<>

        public double Density { get; set; }

        public double Viscosity { get; set; }

        public double[] Lambda { get; private set; }

        internal CubicKernel3d Kernel { get; private set; }

        internal ParticleHash3d Hash { get; private set; }

        internal double[] Densities { get; private set; }

        public int NumParticles { get { return Positions.Count; } }

        public int NumConstraints { get { return Constraints.Count; } }

        public double Dampning { get; set; }

        public double ParticleRadius { get; protected set; }

        public double ParticleDiameter { get { return ParticleRadius * 2.0; } }

        public double ParticleMass { get; protected set; }

        public List<Vector3d> Positions { get; set; }

        public List<Vector3d> Predicted { get; set; }

        public List<Vector3d> Velocities { get; set; }

        public Box3d Bounds { get; private set; }

        public List<Constraint3d> Constraints { get; set; }

        private List<StaticConstraint3d> StaticConstraints { get; set; }

        public Body3d(int numParticles, double radius, double mass)
        {
            InitBody3d(numParticles, radius, mass);
        }

        public Body3d(ParticleSource source, double radius, double density, Matrix4x4d RTS)
        {
            InitBody3d(source.NumParticles, radius, 1.0);
            Density = density;
            Viscosity = 0.02;
            Dampning = 0;

            double d = ParticleDiameter;
            ParticleMass = 0.8 * d * d * d * Density;

            CreateParticles(source, RTS);
            InitFluidAttr(NumParticles);

        }

        private void InitBody3d(int numParticles, double radius, double mass)
        {
            Positions = Enumerable.Repeat(new Vector3d(), numParticles).ToList();
            Predicted = Enumerable.Repeat(new Vector3d(), numParticles).ToList();
            Velocities = Enumerable.Repeat(new Vector3d(), numParticles).ToList();
            Constraints = new List<Constraint3d>();
            StaticConstraints = new List<StaticConstraint3d>();

            ParticleRadius = radius;
            ParticleMass = mass;
            Dampning = 1;

            if (ParticleMass <= 0)
                throw new ArgumentException("Particles mass <= 0");

            if (ParticleRadius <= 0)
                throw new ArgumentException("Particles radius <= 0");
        }

        private void InitFluidAttr(int numParticles)
        {
            double cellSize = ParticleRadius * 4.0;
            Kernel = new CubicKernel3d(cellSize);
            Hash = new ParticleHash3d(numParticles, cellSize);

            Lambda = new double[numParticles];
            Densities = new double[numParticles];
        }

        public Body3d ContactBody3d(Body3d body)
        {
            int newNumParticles = this.NumParticles + body.NumParticles;
            InitFluidAttr(newNumParticles);

            Debug.Log("11body1 particles: " + this.NumParticles + ", body2 particles: " + body.NumParticles);
            Body3d newBody = this;
            newBody.Positions.AddRange(body.Positions);

            //Debug.Log("22body1 particles: " + this.NumParticles + ", body2 particles: " + body.NumParticles);

            newBody.Predicted.AddRange(body.Predicted);
            newBody.Velocities.AddRange(body.Velocities);

            newBody.Constraints.AddRange(body.Constraints);
            newBody.StaticConstraints.AddRange(body.StaticConstraints);

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

                Positions[i] += new Vector3d(rx, ry, rz) * amount;
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
                if (bounds.Contains(Positions[i]))
                {
                    StaticConstraints.Add(new StaticConstraint3d(this, i));
                }
            }
        }

        public void UpdateBounds()
        {
            Vector3d min = new Vector3d(double.PositiveInfinity);
            Vector3d max = new Vector3d(double.NegativeInfinity);

            for (int i = 0; i < NumParticles; i++)
            {
                min.Min(Positions[i]);
                max.Max(Positions[i]);
            }

            min -= ParticleRadius;
            max += ParticleRadius;

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
                Densities[i] = 0.0;
            }
        }

        public void RandomizePositionOrder(System.Random rnd)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                Vector3d tmp = Positions[i];

                int idx = rnd.Next(0, NumParticles - 1);
                Positions[i] = Positions[idx];
                Positions[idx] = tmp;
            }
            //Predicted = Positions;
            //Array.Copy(Positions, Predicted, NumParticles);
        }

        internal void ComputeViscosity()
        {
            int[,] neighbors = Hash.Neighbors;
            int[] numNeighbors = Hash.NumNeighbors;

            double viscosityMulMass = Viscosity * ParticleMass;

            // Compute viscosity forces (XSPH) 
            for (int i = 0; i < NumParticles; i++)
            {
                //Viscosity for particle Pi. Modifies the velocity.
                //Vi = Vi + c * SUMj Vij * W(Pi - Pj, h)
                Vector3d pi = Predicted[i];

                for (int j = 0; j < numNeighbors[i]; j++)
                {
                    int neighborIndex = neighbors[i, j];
                    if (neighborIndex < NumParticles) // Test if fluid particle
                    {
                        double invDensity = 1.0 / Densities[neighborIndex];
                        Vector3d pn = Predicted[neighborIndex];

                        double k = Kernel.W(pi.x - pn.x, pi.y - pn.y, pi.z - pn.z) * viscosityMulMass * invDensity;
                        Velocities[i] -= k * (Velocities[i] - Velocities[neighborIndex]);
                    }
                }
            }

        }

        private void CreateParticles(ParticleSource source, Matrix4x4d RTS)
        {

            for (int i = 0; i < NumParticles; i++)
            {
                Vector4d pos = RTS * source.Positions[i].xyz1;
                Positions[i] = new Vector3d(pos.x, pos.y, pos.z);
                Predicted[i] = Positions[i];
            }

        }


    }

}