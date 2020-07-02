using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Constraints
{

    public class DistanceConstraint3d : Constraint3d
    {

        private double RestLength;

        private double CompressionStiffness;

        private double StretchStiffness;

        private readonly int i0, i1;

        internal DistanceConstraint3d(Body3d body, int i0, int i1, double stiffness) : base(body)
        {
            this.i0 = i0;
            this.i1 = i1;

            CompressionStiffness = stiffness;
            StretchStiffness = stiffness;
            RestLength = (Body.Particles[i0].Position - Body.Particles[i1].Position).Magnitude;
        }

        internal override void ConstrainPositions(double di)
        {
            double mass = Body.Particles[0].ParticleMass;
            double invMass = 1.0 / mass;
            double sum = mass * 2.0;

            Vector3d n = Body.Particles[i1].Predicted - Body.Particles[i0].Predicted;
            double d = n.Magnitude;
            n.Normalize();

            Vector3d corr;
            if (d < RestLength)
                corr = CompressionStiffness * n * (d - RestLength) * sum;
            else
                corr = StretchStiffness * n * (d - RestLength) * sum;

            Body.Particles[i0].Predicted += invMass * corr * di;

            Body.Particles[i1].Predicted -= invMass * corr * di;

        }

    }

}
