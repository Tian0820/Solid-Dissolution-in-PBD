using System;
using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{
    internal class FluidSolidContact3d : CollisionContact3d
	{
		private Body3d FluidBody, SolidBody;

		private int i0, i1;

		private double Diameter, Diameter2;

		private double Mass0, Mass1;

		internal FluidSolidContact3d(Body3d fluidBody, int i0, Body3d solidBody, int i1)
		{
			FluidBody = fluidBody;
			this.i0 = i0;

            SolidBody = solidBody;
			this.i1 = i1;

			Diameter = FluidBody.Particles[0].ParticleRadius + SolidBody.Particles[0].ParticleRadius;
			Diameter2 = Diameter * Diameter;

			double sum = FluidBody.Particles[0].ParticleMass + SolidBody.Particles[0].ParticleMass;
			Mass0 = FluidBody.Particles[0].ParticleMass / sum;
			Mass1 = SolidBody.Particles[0].ParticleMass / sum;
		}

		internal override void ResolveContact(double di)
		{
            Vector3d normal = FluidBody.Particles[i0].Predicted - SolidBody.Particles[i1].Predicted;

            double sqLen = normal.SqrMagnitude;

            if (sqLen <= Diameter2 && sqLen > 1e-9)
            {
                double len = Math.Sqrt(sqLen);
                normal /= len;

                Vector3d delta = di * (Diameter - len) * normal;

                FluidBody.Particles[i0].Predicted += delta * Mass0;
                FluidBody.Particles[i0].Position += delta * Mass0;

                SolidBody.Particles[i1].Predicted -= delta * Mass1;
                SolidBody.Particles[i1].Position -= delta * Mass1;
            }
        }

    }
}
