using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{

    internal abstract class CollisionContact3d
    {

        internal virtual void ResolveContact(double di)
        {

        }

        internal virtual void ResolveContact(double di, bool isActive)
        {

        }

    }

}