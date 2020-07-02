using System;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;

using PositionBasedDynamics.Collisions;
using PositionBasedDynamics.Constraints;
using PositionBasedDynamics.Sources;

namespace PositionBasedDynamics.Bodies
{

    public class Body3d
    {

        public double Viscosity { get; set; }

        public List<int> Indices { get; private set; }

        public int NumParticles { get { return Particles.Count; } }

        public int NumConstraints { get { return Constraints.Count; } }

        public double Dampning { get; set; }

        public List<Particle> Particles { get; set; }

        public Box3d Bounds { get; private set; }

        //public Dictionary<String, Constraint3d> Constraints { get; set; }
        public List<Constraint3d> Constraints { get; set; }

        public List<StaticConstraint3d> StaticConstraints { get; set; }

        public Body3d(int numParticles, double radius, double mass)
        {
            InitBody3d();
        }

        public void InitBody3d()
        {
            Particles = new List<Particle>();
            //Constraints = new Dictionary<string, Constraint3d>();
            Constraints = new List<Constraint3d>();
            StaticConstraints = new List<StaticConstraint3d>();
            Dampning = 1;
        }

        internal void ConstrainPositions(double di)
        {
            //foreach (string key in Constraints.Keys)
            //{
            //    Debug.Log("ConstrainPositions " + key);
            //}
            //Debug.Log("=====");
            for (int i = 0; i < Constraints.Count; i++)
            {
                //Constraints.Values.ToList()[i].ConstrainPositions(di);
                Constraints[i].ConstrainPositions(di);
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainPositions(di);
            }
        }

        internal void ConstrainVelocities()
        {

            for (int i = 0; i < Constraints.Count; i++)
            {
                //Constraints.Values.ToList()[i].ConstrainVelocities();
                Constraints[i].ConstrainVelocities();
            }

            for (int i = 0; i < StaticConstraints.Count; i++)
            {
                StaticConstraints[i].ConstrainVelocities();
            }

        }

        public void RandomizePositions(System.Random rnd, double amount)
        {
            for(int i = 0; i < NumParticles; i++)
            {
                double rx = rnd.NextDouble() * 2.0 - 1.0;
                double ry = rnd.NextDouble() * 2.0 - 1.0;
                double rz = rnd.NextDouble() * 2.0 - 1.0;

                Particles[i].Position += new Vector3d(rx, ry, rz) * amount;
            }
        }

        public void MarkAsStatic(List<Vector3d> bounds)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                if (bounds.Contains(Particles[i].Position))
                {
                    //Debug.Log("static: " + i);
                    StaticConstraint3d staticConstraint = new StaticConstraint3d(this, i);
                    StaticConstraints.Add(staticConstraint);
                }
            }
        }

        public void UpdateBounds()
        {
            Vector3d min = new Vector3d(double.PositiveInfinity);
            Vector3d max = new Vector3d(double.NegativeInfinity);

            List<Vector3d> postions = new List<Vector3d>();
            for(int i = 0; i < Particles.Count; i++)
            {
                postions.Add(Particles[i].Position);
            }

            for (int i = 0; i < NumParticles; i++)
            {
                min.Min(postions[i]);
                max.Max(postions[i]);
            }

            if (Particles.Count > 0)
            {
                double radius = Particles[0].ParticleRadius;
                min -= radius;
                max += radius;
            }

            Bounds = new Box3d(min, max);
        }

        

        public void UpdateConstrains()
        {
            //FluidConstraint3d constraint = (FluidConstraint3d)Constraints["FluidConstraint3d"];
            //if(constraint == null)
            //    throw new InvalidOperationException("there is no fluid constraint");

            //constraint.UpdateBody(this);
        }

        public void RandomizePositionOrder(System.Random rnd)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                Vector3d tmp = Particles[i].Position;

                int idx = rnd.Next(0, NumParticles - 1);
                Particles[i].Position = Particles[idx].Position;
                Particles[idx].Position = tmp;
            }
        }

        protected void CreateParticles(ParticleSource source, Matrix4x4d RTS)
        {
            for (int i = 0; i < NumParticles; i++)
            {
                Vector4d pos = RTS * source.Positions[i].xyz1;
                Particles[i].Position = new Vector3d(pos.x, pos.y, pos.z);
                Particles[i].Predicted = Particles[i].Position;
            }

        }

    }

}
