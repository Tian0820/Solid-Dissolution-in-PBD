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
    public class FluidBody3d : Body3d
    {
        internal CubicKernel3d FluidKernel { get; private set; }

        internal ParticleHash3d FluidHash { get; private set; }

        public List<double> Lambda { get; private set; }

        public FluidBody3d(ParticlePhase phase, ParticleSource source, double radius, double density, Matrix4x4d RTS)
            : base(source.NumParticles, radius, 1.0)
        {
            InitBody3d();
            for (int i = 0; i < source.NumParticles; i++)
            {
                double d = radius * 2;
                Particle newParticle = new Particle(i, radius, 0.8 * d * d * d * density, density, ParticlePhase.FLUID);
                Particles.Add(newParticle);
            }
            Viscosity = 0.02;
            Dampning = 0;


            CreateParticles(source, RTS);
            InitFluidAttr();
        }

        private void InitFluidAttr()
        {
            double cellSize = Particles[0].ParticleRadius * 4.0;
            FluidKernel = new CubicKernel3d(cellSize);

            FluidHash = new ParticleHash3d(NumParticles, cellSize);

            Lambda = Enumerable.Repeat(0.0, NumParticles).ToList();
        }

        internal void Reset()
        {
            for (int i = 0; i < NumParticles; i++)
            {
                Lambda[i] = 0.0;
                Particles[i].DynamicDensity = 0.0;
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

                        //Debug.Log("????" + new Vector3d(pi.x - pn.x, pi.y - pn.y, pi.z - pn.z));
                        double k = FluidKernel.W(pi.x - pn.x, pi.y - pn.y, pi.z - pn.z) * viscosityMulMass * invDensity;
                        Particles[i].Velocity -= k * (Particles[i].Velocity - Particles[neighborIndex].Velocity);
                    }
                }
            }
        }

        public void ContactParticle(List<Particle> particles)
        {
            for (int i = 0; i < particles.Count; i++)
            {
                if (!FindTransedParticle(particles[i].Index))
                {
                    particles[i].Phase = ParticlePhase.TRANSED;
                    particles[i].Velocity = new Vector3d(0, -1, 0);
                    particles[i].DynamicDensity = 0.0;
                    particles[i].StaticDensity = Particles[0].StaticDensity;
                    //TODO modify mass
                    particles[i].ParticleMass = 90;
                    particles[i].ParticleRadius = Particles[0].ParticleRadius;
                    particles[i].Color = new Vector4d(1, 0, 0, 0.1f);

                    Lambda.Add(0.0);
                    Particles.Add(particles[i]);
                }
            }
            UpdateConstrains();
        }

        public void AddBoundary(FluidBoundary3d boundary)
        {
            FluidConstraint3d constraint = new FluidConstraint3d(this, boundary);
            //Constraints["FluidConstraint3d"] = constraint;
            Constraints.Add(constraint);
        }

        private bool FindTransedParticle(int index)
        {
            bool exist = false;
            for (int i = 0; i < NumParticles; i++)
            {
                if (Particles[i].Index == index && Particles[i].Phase.Equals(ParticlePhase.TRANSED))
                {
                    exist = true;
                    break;
                }
            }
            return exist;
        }

        //public Body3d ContactBody3d(Body3d body)
        //{
        //    Body3d newBody = this;

        //    newBody.Particles.AddRange(body.Particles);

        //    //newBody.Constraints.AddRange(body.Constraints);
        //    //newBody.StaticConstraints.AddRange(body.StaticConstraints);
        //    newBody.FluidHash = new ParticleHash3d(newBody.NumParticles, newBody.Particles[0].ParticleRadius * 4.0);
        //    newBody.Lambda = Enumerable.Repeat(0.0, newBody.NumParticles).ToList();

        //    return newBody;
        //}


    }
}
