using UnityEngine;
using System;
using System.Collections.Generic;

using Common.Mathematics.LinearAlgebra;

namespace PositionBasedDynamics.Collisions
{

    public class ParticleHash3d
    {

        private class HashEntry
        {
            public int TimeStamp;

            readonly public List<int> Indices;

            public HashEntry(int capacity, int timeStamp)
            {
                Indices = new List<int>(capacity);
                TimeStamp = timeStamp;
            }
        }

        //private int[,] Neighbors;

        //private int[] NumNeighbors;

        //public int numParticles { get; private set; }

        public int MaxNeighbors { get; set; }

        private int MaxParticlesPerCell { get; set; }

        private int[,] Indexes { get; set; }

        private double CellSize { get; set; }

        private double InvCellSize { get; set; }

        private int CurrentTimeStamp { get; set; }

        private Dictionary<int, HashEntry> GridMap { get; set; }

        public ParticleHash3d(int numParticles, double cellSize, int maxNeighbors = 200, int maxParticlesPerCell = 50)
        {

            //this.numParticles = numParticles;
            MaxParticlesPerCell = maxParticlesPerCell;
            MaxNeighbors = maxNeighbors;
            CurrentTimeStamp = 0;

            CellSize = cellSize;
            InvCellSize = 1.0 / CellSize;

            GridMap = new Dictionary<int, HashEntry>();

            //NumNeighbors = new int[NumParticles];
            //Neighbors = new int[NumParticles, MaxNeighbors];

            Indexes = new int[27, 3];

            for (int x = 0; x < 3; x++)
            {
                for (int y = 0; y < 3; y++)
                {
                    for (int z = 0; z < 3; z++)
                    {
                        int i = x + y * 3 + z * 9;

                        Indexes[i, 0] = x;
                        Indexes[i, 1] = y;
                        Indexes[i, 2] = z;
                    }
                }
            }
        }

        public void IncrementTimeStamp()
        {
            CurrentTimeStamp++;
        }

        int Floor(double v)
        {
            return (int)(v * InvCellSize + 32768.1) - 32768;
        }

        void Floor(Vector3d v, out int pos1, out int pos2, out int pos3)
        {
            pos1 = (int)(v.x * InvCellSize + 32768.1) - 32768;
            pos2 = (int)(v.y * InvCellSize + 32768.1) - 32768;
            pos3 = (int)(v.z * InvCellSize + 32768.1) - 32768;
        }

        int Hash(int x, int y, int z)
        {
            int p1 = 73856093 * x;
            int p2 = 19349663 * y;
            int p3 = 83492791 * z;
            return p1 + p2 + p3;
        }

        int Hash(Vector3d particle)
        {
            int x = (int)(particle.x * InvCellSize + 32768.1) - 32768 + 1;
            int y = (int)(particle.y * InvCellSize + 32768.1) - 32768 + 1;
            int z = (int)(particle.z * InvCellSize + 32768.1) - 32768 + 1;

            int p1 = 73856093 * x;
            int p2 = 19349663 * y;
            int p3 = 83492791 * z;
            return p1 + p2 + p3;
        }

        void AddToGrid(int i, Vector3d particle)
        {

            int cellPos = Hash(particle);
            HashEntry entry = null;

            if (GridMap.TryGetValue(cellPos, out entry))
            {
                if (entry.TimeStamp != CurrentTimeStamp)
                {
                    entry.TimeStamp = CurrentTimeStamp;
                    entry.Indices.Clear();
                }
            }
            else
            {
                entry = new HashEntry(MaxParticlesPerCell, CurrentTimeStamp);
                GridMap.Add(cellPos, entry);
            }

            entry.Indices.Add(i);
        }

        public List<Particle> NeighborhoodSearch(List<Particle> particles)
        {
            //if (particles.Count > numParticles)
            //    throw new ArgumentException("Particle array length larger than expected");
            int numParticles = particles.Count;

            List<Vector3d> particlePos = new List<Vector3d>();
            for (int i = 0; i < particles.Count; i++)
            {
                particlePos.Add(particles[i].Position);
            }

            double r2 = CellSize * CellSize;

            for (int i = 0; i < numParticles; i++)
            {
                AddToGrid(i, particlePos[i]);
            }

            for (int i = 0; i < numParticles; i++)
            {
                //NumNeighbors[i] = 0;
                particles[i].NeighbourIndexes = new List<int>();

                Vector3d p0 = particlePos[i];

                int cellPos1, cellPos2, cellPos3;
                Floor(p0, out cellPos1, out cellPos2, out cellPos3);

                for (int j = 0; j < 27; j++)
                {
                    int cellPos = Hash(cellPos1 + Indexes[j, 0], cellPos2 + Indexes[j, 1], cellPos3 + Indexes[j, 2]);

                    HashEntry entry = null;
                    GridMap.TryGetValue(cellPos, out entry);

                    if (entry != null && entry.TimeStamp == CurrentTimeStamp)
                    {
                        int count = entry.Indices.Count;
                        for (int m = 0; m < count; m++)
                        {
                            int pi = entry.Indices[m];
                            if (pi == i) continue;

                            Vector3d p;
                            p.x = p0.x - particlePos[pi].x;
                            p.y = p0.y - particlePos[pi].y;
                            p.z = p0.z - particlePos[pi].z;

                            double dist2 = p.x * p.x + p.y * p.y + p.z * p.z;

                            if (dist2 < r2)
                            {
                                if (particles[i].NeighbourIndexes.Count < MaxNeighbors)
                                {
                                    //Neighbors[i, NumNeighbors[i]++] = pi;
                                    particles[i].NeighbourIndexes.Add(pi);
                                }
                                else
                                    throw new InvalidOperationException("too many neighbors detected");
                            }
                        }
                    }
                }
            }
            return particles;
            //end of function
        }

        //public void NeighborhoodSearch(List<Vector3d> particles, List<Vector3d> boundary)
        public List<Particle> NeighborhoodSearch(List<Particle> particles1, List<Particle> particles2)
        {

            //Debug.Log("particles.Count" + particles.Count);
            //Debug.Log("NumParticles" + NumParticles);
            //if (particles1.Count > numParticles)
            //    throw new ArgumentException("Particle array length larger than expected");
            int numParticles = particles1.Count;

            List<Vector3d> particlePos1 = new List<Vector3d>();
            List<Vector3d> particlePos2 = new List<Vector3d>();
            for (int i = 0; i < particles1.Count; i++)
            {
                particlePos1.Add(particles1[i].Position);
            }
            for (int i = 0; i < particles2.Count; i++)
            {
                particlePos2.Add(particles2[i].Position);
            }

            //double invCellSize = 1.0 / CellSize;
            double r2 = CellSize * CellSize;

            for (int i = 0; i < numParticles; i++)
            {
                AddToGrid(i, particlePos1[i]);
            }

            for (int i = 0; i < particles2.Count; i++)
            {
                AddToGrid(numParticles + i, particlePos2[i]);
            }

            for (int i = 0; i < numParticles; i++)
            {
                Vector3d p0 = particlePos1[i];
                //NumNeighbors[i] = 0;
                particles1[i].NeighbourIndexes = new List<int>();

                int cellPos1, cellPos2, cellPos3;
                Floor(particlePos1[i], out cellPos1, out cellPos2, out cellPos3);

                for (int j = 0; j < 27; j++)
                {
                    int cellPos = Hash(cellPos1 + Indexes[j, 0], cellPos2 + Indexes[j, 1], cellPos3 + Indexes[j, 2]);

                    HashEntry entry = null;
                    GridMap.TryGetValue(cellPos, out entry);

                    if (entry != null && entry.TimeStamp == CurrentTimeStamp)
                    {
                        int count = entry.Indices.Count;
                        for (int m = 0; m < count; m++)
                        {
                            int pi = entry.Indices[m];
                            if (pi == i) continue;

                            double dist2 = 0.0;

                            if (pi < numParticles)
                            {
                                Vector3d p;
                                p.x = p0.x - particlePos1[pi].x;
                                p.y = p0.y - particlePos1[pi].y;
                                p.z = p0.z - particlePos1[pi].z;

                                dist2 = p.x * p.x + p.y * p.y + p.z * p.z;
                            }
                            else
                            {
                                int bi = pi - numParticles;

                                Vector3d p;
                                p.x = p0.x - particlePos2[bi].x;
                                p.y = p0.y - particlePos2[bi].y;
                                p.z = p0.z - particlePos2[bi].z;

                                dist2 = p.x * p.x + p.y * p.y + p.z * p.z;
                            }

                            if (dist2 < r2)
                            {
                                if ((pi < numParticles && particles1[pi].Phase.Equals(ParticlePhase.FLUID)) ||
                                    (pi >= numParticles && particles2[pi - numParticles].Phase.Equals(ParticlePhase.BOUNDARY)))
                                {
                                    if (particles1[i].NeighbourIndexes.Count < MaxNeighbors)
                                    {
                                        //Neighbors[i, NumNeighbors[i]++] = pi;
                                        particles1[i].NeighbourIndexes.Add(pi);
                                    }
                                    else
                                        throw new InvalidOperationException("too many neighbors detected");
                                }
                            }
                        }
                    }
                }
            }
            return particles1;
            //End of function
        }
    }

}