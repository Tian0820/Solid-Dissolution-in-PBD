using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Constraints
{

    public class BendingConstraint3d : Constraint3d
    {

        private double RestLength { get; set; }

        private double Stiffness { get; set; }

        private readonly int i0, i1, i2;

        internal BendingConstraint3d(Body3d body, int i0, int i1, int i2, double stiffness) : base(body)
        {
            this.i0 = i0;
            this.i1 = i1;
            this.i2 = i2;

            Stiffness = stiffness;

            Vector3d center = (Body.Particles[i0].Position + Body.Particles[i1].Position + Body.Particles[i2].Position) / 3.0;
            RestLength = (Body.Particles[i2].Position - center).Magnitude;
        }

        internal override void ConstrainPositions(double di)
        {

            Vector3d center = (Body.Particles[i0].Predicted + Body.Particles[i1].Predicted + Body.Particles[i2].Predicted) / 3.0;

            Vector3d dirCenter = Body.Particles[i2].Predicted - center;

            double distCenter = dirCenter.Magnitude;
            double diff = 1.0 - (RestLength / distCenter);
            double mass = Body.Particles[0].ParticleMass;

            double w = mass + mass * 2.0f + mass;

            Vector3d dirForce = dirCenter * diff;

            Vector3d fa = Stiffness * (2.0 * mass / w) * dirForce * di;
            Body.Particles[i0].Predicted += fa;

            Vector3d fb = Stiffness * (2.0 * mass / w) * dirForce * di;
            Body.Particles[i1].Predicted += fb;

            Vector3d fc = -Stiffness * (4.0 * mass / w) * dirForce * di;
            Body.Particles[i2].Predicted += fc;

        }

    }

}
