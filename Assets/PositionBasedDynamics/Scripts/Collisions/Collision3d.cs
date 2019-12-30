using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{

    public abstract class Collision3d
    {
        internal virtual bool FindContacts(IList<Body3d> bodies, List<CollisionContact3d> contacts)
        {
            return false;
        }

        internal virtual void FindContacts(Body3d body1, Body3d body2, List<CollisionContact3d> contacts)
        {

        }
    }

}