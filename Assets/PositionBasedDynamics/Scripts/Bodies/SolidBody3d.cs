using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Sources;

namespace PositionBasedDynamics.Bodies
{
    public class SolidBody3d : Body3d
    {
        public double Stiffness { get; private set; }

        public SolidBody3d(ParticlePhase phase, ParticleSource source, double radius, double mass, Matrix4x4d RTS)
            : base(source.NumParticles, radius, mass)
        {
            InitBody3d();
            for (int i = 0; i < source.NumParticles; i++)
            {
                Particle newParticle = new Particle(i, radius, mass, 0, phase);
                Particles.Add(newParticle);
            }

            Stiffness = 1.0;

            CreateParticles(source, RTS);
            //Constraints["ShapeMatchingConstraint3d"] = new ShapeMatchingConstraint3d(this, particleAttr, Stiffness);
        }
    }
}