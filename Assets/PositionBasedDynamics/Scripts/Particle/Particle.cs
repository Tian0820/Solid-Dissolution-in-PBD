using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;

using PositionBasedDynamics.Constraints;

namespace PositionBasedDynamics
{
    public class Particle
    {
        public Vector3d Position { get; set; }
        public Vector3d Predicted { get; set; }
        public Vector3d Velocity { get; set; }
        public Vector3d F { get; set; }
        public double Imass, Tmass, SFriction, KFriction, t; // inverse mass, temporary height-scaled mass, coeffs of friction
        public int Bod { get; set; } // body (if any) this particle belongs to, for disabling collisions
        public Phase phase { get; set; } // phase of this particle

        public Particle(Vector3d position, double mass, Phase phase = Phase.SOLID)
        {
            Position = position;
            Velocity = new Vector3d();
            Initialize(mass);
        }

        public enum Phase
        {
            FLUID,
            SOLID,
            NUM_PHASES
        }

        public enum ConstraintGroup
        {
            STABILIZATION,
            CONTACT,
            STANDARD,
            SHAPE,
            NUM_CONSTRAINT_GROUPS
        }

        public void Initialize(double mass)
        {
            t = 4;
            Predicted = new Vector3d();
            Bod = -1;

            if (mass <= 0)
            {
                Imass = -mass;
            }
            else
            {
                Imass = 1 / mass;
            }
            Tmass = Imass;

            F = new Vector3d();
            SFriction = 0;
            KFriction = 0; // usually smaller the coefficient of static friction
        }

        void setStatic() { Imass = 0; }

        Vector3d Guess(double seconds)
        {
            return Imass == 0.0 ? Position : Position + seconds * Velocity;
        }

        void ScaleMass()
        {
            if (Imass != 0.0)
            {
                Tmass = 1.0 / ((1.0 / Imass) * Math.Exp(-Position.y));
            }
            else
            {
                Tmass = 0.0;
            }
        }
    }


}
