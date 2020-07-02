using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Bodies;

namespace PositionBasedDynamics.Collisions
{
    public class ParticleCollision3d : Collision3d
    {

        private double Distance { get; set; }

        public ParticleCollision3d(float distance)
        {
            Distance = distance;
        }

        internal override void FindContacts(Body3d body1, Body3d body2, List<CollisionContact3d> contacts)
        {
            if (body1 != null && body2 != null)
            {
                int numParticles1 = body1.NumParticles;
                int numParticles2 = body2.NumParticles;
                double radius = body1.Particles[0].ParticleRadius + body2.Particles[0].ParticleRadius;

                for (int i0 = 0; i0 < numParticles1; i0++)
                {
                    for (int i1 = 0; i1 < numParticles2; i1++)
                    {
                        double d = Vector3d.Distance(body1.Particles[i0].Predicted, body2.Particles[i1].Predicted) + Distance - radius;
                        if (d < 0.0)
                        {
                            //contacts.Add(new FluidSolidContact3d(body1, i0, body2, i1));
                            ////if (body1.Particles[i0].Phase.Equals(ParticlePhase.SOLID) && body2.Particles[i1].Phase.Equals(ParticlePhase.FLUID))
                            //    body1.Particles[i0].needTrans = true;
                            if (body1.Particles[i0].Phase.Equals(ParticlePhase.CLOTH) && body2.Particles[i1].Phase.Equals(ParticlePhase.FLUID))
                            {
                                //Debug.Log("contact");
                                contacts.Add(new FluidClothContact3d(body1, i0, body2, i1));
                            }
                        }
                    }
                }
            }

        }
    }
}

