using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;

namespace PositionBasedDynamics
{
    public abstract class UnifiedConstraint3d
    {
        double Stiffness;

        public UnifiedConstraint3d()
        {
            Stiffness = 1;
        }

        internal virtual void Project(List<Particle> estimates, int[] counts)
        {

        }

        internal virtual double Evaluate(List<Particle> estimates)
        {
            return 0;
        }

        internal virtual Vector2d Gradient(List<Particle> estimates, int respect)
        {
            return new Vector2d();
        }

        internal virtual void UpdateCounts(int counts)
        {

        }
    }
}
