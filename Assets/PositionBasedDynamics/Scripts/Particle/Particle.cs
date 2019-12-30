using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

namespace PositionBasedDynamics
{
    public class Particle
    {
        public Vector3d Position { get; set; }

        public Vector3d Predicted { get; set; }

        public Vector3d Velocity { get; set; }

        public double StaticDensity { get; set; }

        public double DynamicDensity { get; set; }

        public double ParticleRadius { get; set; }

        public double ParticleDiameter { get { return ParticleRadius * 2.0; } }

        public double ParticleMass { get; set; }

        public double BoundaryPsi { get; set; }

        //public int[] NeighbourIndexes { get; set; }

        public List<int> NeighbourIndexes { get; set; }

        public ParticlePhase Phase { get; set; } // phase of this particle

        public bool needTrans { get; set; }

        public Particle(Vector3d position, Vector3d predicted, Vector3d velocity, double radius,
            double mass, double staticDensity, ParticlePhase phase)
        {
            Position = position;
            Predicted = predicted;
            Velocity = velocity;
            Phase = phase;
            ParticleRadius = radius;
            ParticleMass = mass;
            StaticDensity = staticDensity;
            DynamicDensity = 0.0;
            //NeighbourIndexes = new int[MaxNeighbourNum];
            NeighbourIndexes = new List<int>();
            needTrans = false;
            if (ParticleMass <= 0)
                throw new ArgumentException("Particles mass <= 0");

            if (ParticleRadius <= 0)
                throw new ArgumentException("Particles radius <= 0");
        }

        public Particle(double radius, double mass, double staticDensity, ParticlePhase phase)
        {
            Position = new Vector3d(0, 0, 0);
            Predicted = new Vector3d(0, 0, 0);
            Velocity = new Vector3d(0, 0, 0);
            Phase = phase;
            ParticleRadius = radius;
            ParticleMass = mass;
            StaticDensity = staticDensity;
            DynamicDensity = 0.0;
            //NeighbourIndexes = new int[MaxNeighbourNum];
            NeighbourIndexes = new List<int>();
            needTrans = false;
            if (ParticleMass <= 0)
                throw new ArgumentException("Particles mass <= 0");

            if (ParticleRadius <= 0)
                throw new ArgumentException("Particles radius <= 0");
        }

        public Particle(double radius, double staticDensity, ParticlePhase phase)
        {
            Position = new Vector3d(0, 0, 0);
            Predicted = new Vector3d(0, 0, 0);
            Velocity = new Vector3d(0, 0, 0);
            Phase = phase;
            ParticleRadius = radius;
            StaticDensity = staticDensity;
            DynamicDensity = 0.0;
            //NeighbourIndexes = new int[MaxNeighbourNum];
            NeighbourIndexes = new List<int>();
            needTrans = false;
            if (ParticleRadius <= 0)
                throw new ArgumentException("Particles radius <= 0");
        }

    }

}
