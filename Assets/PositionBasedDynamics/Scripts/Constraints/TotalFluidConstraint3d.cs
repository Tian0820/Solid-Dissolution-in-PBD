using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;

namespace PositionBasedDynamics.Constraints
{
    public class TotalFluidConstraint3d : UnifiedConstraint3d
    {
        List<int>[] Neighbors;
        List<int> ps;
        Vector3d[] deltas;
        double p0;
        Dictionary<int, double> lambdas;

        private int numParticles;


        public TotalFluidConstraint3d()
        {
        }

        public TotalFluidConstraint3d(double density, List<int> particles)
        {
            p0 = density;
            Neighbors = new List<int>[particles.Count];
            deltas = new Vector3d[particles.Count];
            numParticles = particles.Count;

            for (int i = 0; i < particles.Count; i++)
            {
                ps.Add(particles[i]);
            }
        }

        public void AddParticle(int index)
        {
            numParticles++;
            Neighbors = new List<int>[numParticles];
            deltas = new Vector3d[numParticles];
            ps.Add(index);
        }

        public void RemoveParticle(int index)
        {
            numParticles--;
            Neighbors = new List<int>[numParticles];
            deltas = new Vector3d[numParticles];
            ps.RemoveAt(index);
        }

        internal override void Project(List<Particle> estimates, int[] counts)
        {
            // Find neighboring particles and estimate pi for each particle
            lambdas.Clear();
            for (int k = 0; k < ps.Count; k++)
            {
                Neighbors[k].Clear();
                int i = ps[k];
                Particle p_i = estimates[i];
                double pi = 0.0, denom = 0.0;

                // Find neighbors
                for (int j = 0; j < estimates.Count; j++)
                {
                    // Check if the next particle is actually this particle
                    if (j != i)
                    {
                        Particle p_j = estimates[j];

                        // Ignore fixed particles
                        if (p_j.Imass == 0) continue;
                        Vector3d r = p_i.Predicted - p_j.Predicted;
                        double rlen2 = Vector3d.Dot(r, r);
                        if (rlen2 < H2)
                        {
                            // Found a neighbor! Remember it and add to pi and the gamma denominator
                            Neighbors[k].Add(j);
                            double incr = Poly6(rlen2) / p_j.Imass;
                            if (p_j.phase == Particle.Phase.SOLID)
                            {
                                incr *= S_SOLID;
                            }
                            pi += incr;

                            Vector3d gr_in = Grad(estimates, k, j);
                            denom += Vector3d.Dot(gr_in, gr_in);
                        }

                        // If it is, cut to the chase
                    }
                    else
                    {
                        Neighbors[k].Add(j);
                        pi += Poly6(0) / p_i.Imass;
                    }
                }

                Vector3d gr_out = Grad(estimates, k, i);
                denom += Vector3d.Dot(gr_out, gr_out);

                // Compute the gamma value
                double lambda = -((pi / p0) - 1.0) / (denom + RELAXATION);
                lambdas[i] = lambda;
            }

            // Compute actual deltas
            for (int k = 0; k < ps.Count; k++)
            {
                Vector3d delta = new Vector3d();
                int i = ps[k];
                Particle p_i = estimates[i];

                for (int x = 0; x < Neighbors[k].Count; x++)
                {
                    int j = Neighbors[k][x];
                    if (i == j) continue;
                    Particle p_j = estimates[j];
                    Vector3d r = p_i.Predicted - p_j.Predicted;
                    double rlen = r.Magnitude;
                    Vector3d sg = SpikyGrad(r, rlen);
                    double lambdaCorr = -K_P * Mathf.Pow((float)(Poly6(rlen * rlen) / Poly6(DQ_P * DQ_P * H * H)), (float)E_P);
                    delta += (lambdas[i] + lambdas[j] + lambdaCorr) * sg;
                }
                deltas[k] = (delta / p0);
            }

            for (int k = 0; k < ps.Count; k++)
            {
                int i = ps[k];
                Particle p_i = estimates[i];
                p_i.Predicted += deltas[k] / ((double)Neighbors[k].Count + counts[i]);
            }

        }

        private double Poly6(double r2)
        {
            if (r2 >= H2) return 0;
            double term2 = (H2 - r2);
            return (315.0 / (64 * Mathf.PI * H9)) * (term2 * term2 * term2);
            //    return (H-r) / (H*H);
        }

        private Vector3d Grad(List<Particle> estimates, int k, int j)
        {
            int i = ps[k];
            Particle p_i = estimates[i];
            Particle p_j = estimates[j];
            Vector3d r = p_i.Predicted - p_j.Predicted;
            double rlen = r.Magnitude;
            if (p_i != p_j)
            {
                return new Vector3d(0, 0, 0) - (SpikyGrad(r, rlen) / (p0));
            }

            Vector3d output = new Vector3d();
            for (int x = 0; x < Neighbors[k].Count; x++)
            {
                p_j = estimates[Neighbors[k][x]];
                r = p_i.Predicted - p_j.Predicted;
                rlen = r.Magnitude;
                output += (p_j.phase == Particle.Phase.SOLID ? S_SOLID : 1.0) * SpikyGrad(r, rlen);
            }

            return output / (p0);
        }

        private Vector3d SpikyGrad(Vector3d r, double rlen2)
        {
            if (rlen2 >= H) return new Vector3d();
            if (rlen2 == 0) return new Vector3d();
            return r.Normalized * (45.0 / (Mathf.PI * H6)) * (H - rlen2) * (H - rlen2);
        }

        private const double H = 2.0;
        private const double H2 = 4.0;
        private const double H6 = 64.0;
        private const double H9 = 512.0;
        // Fluid-solid coupling constant
        private const double S_SOLID = 0.0;
        // Epsilon in gamma correction denominator
        private const double RELAXATION = 0.01;
        // Pressure terms
        private const double K_P = 0.1;
        private const double E_P = 4;
        private const double DQ_P = 0.2;
    }
}