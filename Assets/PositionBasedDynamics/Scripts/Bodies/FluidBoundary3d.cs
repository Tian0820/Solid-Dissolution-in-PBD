using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

using Common.Geometry.Shapes;
using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Collisions;
using PositionBasedDynamics.Sources;

namespace PositionBasedDynamics.Bodies
{

    public class FluidBoundary3d
    {

        //public List<Vector3d> Positions { get; private set; }
        public List<Particle> Particles { get; private set; }

        //public double[] Psi { get; private set; } 

        //public double ParticleRadius { get; private set; }

        //public double ParticleDiameter { get { return ParticleRadius * 2.0; } }

        //public double Density { get; private set; }

        public int NumParticles { get; private set;  }

        public FluidBoundary3d(ParticleSource source, double radius, double density, Matrix4x4d RTS)
        {
            //ParticleRadius = radius;
            //Density = density;

            CreateParticles(source, density, radius, RTS);
            CreateBoundaryPsi();
        }

        private void CreateParticles(ParticleSource source, double density, double radius, Matrix4x4d RTS)
        {
            NumParticles = source.NumParticles;

            //Positions = new List<Vector3d>(NumParticles);
            Particles = new List<Particle>();
            //Particles = Enumerable.Repeat(new Particle(new Vector3d(), new Vector3d(), new Vector3d(),
            //    radius, 1.0, ParticlePhase.BOUNDARY), NumParticles).ToList();

            for (int i = 0; i < NumParticles; i++)
            {
                Vector4d pos = RTS * source.Positions[i].xyz1;
                Particle newParticle = new Particle(i, radius, density, ParticlePhase.BOUNDARY);
                newParticle.Position = new Vector3d(pos.x, pos.y, pos.z);
                Particles.Add(newParticle);
            }

        }

        private void CreateBoundaryPsi()
        {

            //Psi = new double[NumParticles];

            double cellSize = Particles[0].ParticleRadius * 4.0;

            ParticleHash3d hash = new ParticleHash3d(NumParticles, cellSize);

            Particles = hash.NeighborhoodSearch(Particles);
            //hash.NeighborhoodSearch(Particles);

            //int[,] neighbors = hash.Neighbors;
            //int[] numNeighbors = hash.NumNeighbors;
            int[,] neighbors = new int[Particles.Count, hash.MaxNeighbors];
            int[] numNeighbors = new int[Particles.Count];
            for (int i = 0; i < Particles.Count; i++)
            {
                numNeighbors[i] = Particles[i].NeighbourIndexes.Count;
                for (int j = 0; j < numNeighbors[i]; j++)
                {
                    neighbors[i, j] = Particles[i].NeighbourIndexes[j];
                }
            }

            CubicKernel3d kernel = new CubicKernel3d(cellSize);

            for (int i = 0; i < NumParticles; i++)
            {
                double delta = kernel.W_zero;

                for (int j = 0; j < numNeighbors[i]; j++)
                {
                    //int neighborIndex = Particles[i].NeighbourIndexes[j];
                    int neighborIndex = neighbors[i, j];

                    Vector3d p = Particles[i].Position - Particles[neighborIndex].Position;

                    delta += kernel.W(p.x, p.y, p.z);
                }

                double volume = 1.0 / delta;

                //Psi[i] = Density * volume;
                Particles[i].BoundaryPsi = Particles[i].StaticDensity * volume;
            }

        }

    }

}