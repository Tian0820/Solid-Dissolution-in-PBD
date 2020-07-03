using System;
using System.Collections.Generic;
using UnityEngine;

using Common.Geometry.Shapes;
using Common.Mathematics.LinearAlgebra;

using PositionBasedDynamics.Constraints;
using PositionBasedDynamics.Sources;
using PositionBasedDynamics.Utilities;

namespace PositionBasedDynamics.Bodies
{

    public class ClothBody3d : Body3d
    {

        public double StretchStiffness { get; private set; }

        public double BendStiffness { get; private set; }

        public int[] Indices { get; private set; }

        internal CubicKernel3d ClothKernel { get; private set; }

        public double[] Saturations { get; set; }

        private Util Util;

        private int nRows, nCols;

        // cloth
        public ClothBody3d(ParticlePhase phase, TrianglesFromGrid source, double radius, double mass,
            double stretchStiffness, double bendStiffness, Matrix4x4d RTS)
            : base(source.NumParticles, radius, mass)
        {
            InitBody3d();
            Util = new Util();
            nRows = source.Rows;
            nCols = source.Columns;

            for (int i = 0; i < source.NumParticles; i++)
            {
                Particle newParticle = new Particle(i, radius, mass, 0, phase);
                Particles.Add(newParticle);
            }
            StretchStiffness = stretchStiffness;
            BendStiffness = bendStiffness;
            Saturations = new double[NumParticles];
            for (int i = 0; i < NumParticles; i++)
            {
                Saturations[i] = 0.0;
            }

            CreateParticles(source, RTS);
            CreateConstraints();

            InitClothAttr();
            Debug.Log("cloth: " + NumParticles + "; nRows: " + nRows + "; nCols: " + nCols);
        }

        private void InitClothAttr()
        {
            double cellSize = Particles[0].ParticleRadius * 4.0;
            ClothKernel = new CubicKernel3d(cellSize);
        }

        private void CreateConstraints()
        {
            int height = nCols + 1;
            int width = nRows + 1;

            // Horizontal
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < (width - 1); x++)
                {
                    Constraints.Add(new DistanceConstraint3d(this, y * width + x, y * width + x + 1, StretchStiffness));
                }
            }

            // Vertical
            for (int x = 0; x < width; x++)
            {
                for (int y = 0; y < (height - 1); y++)
                {
                    Constraints.Add(new DistanceConstraint3d(this, y * width + x, (y + 1) * width + x, StretchStiffness));
                }
            }


            // Shearing distance constraint
            for (int y = 0; y < (height - 1); y++)
            {
                for (int x = 0; x < (width - 1); x++)
                {
                    Constraints.Add(new DistanceConstraint3d(this, y * width + x, (y + 1) * width + x + 1, StretchStiffness));
                    Constraints.Add(new DistanceConstraint3d(this, (y + 1) * width + x, y * width + x + 1, StretchStiffness));
                }
            }

            //add vertical constraints
            for (int i = 0; i <= nRows; i++)
            {
                for (int j = 0; j < nCols - 1; j++)
                {
                    int i0 = j * (nRows + 1) + i;
                    int i1 = (j + 1) * (nRows + 1) + i;
                    int i2 = (j + 2) * (nRows + 1) + i;

                    Constraints.Add(new BendingConstraint3d(this, i0, i1, i2, BendStiffness));
                }
            }

            //add horizontal constraints
            for (int i = 0; i < nRows - 1; i++)
            {
                for (int j = 0; j <= nCols; j++)
                {
                    int i0 = j * (nRows + 1) + i;
                    int i1 = j * (nRows + 1) + (i + 1);
                    int i2 = j * (nRows + 1) + (i + 2);

                    Constraints.Add(new BendingConstraint3d(this, i0, i1, i2, BendStiffness));
                }
            }

        }

        internal void computeSaturation()
        {
            int height = nCols + 1;
            int width = nRows + 1;
            for (int index = 0; index < NumParticles; index++)
            {
                // particle volume
                double Vi = 1.0 / computeSumW(index);
                
                int m_absorbed = Particles[index].AbsorbedIndexes.Count;
                Saturations[index] = (double)m_absorbed / Vi;

                //Debug.Log("computeSaturation " + index + ": " + m_absorbed + "; Vi: " + Vi + "; Saturation: " + Saturations[index]);
            }
            Saturations = Util.NormalizeData(Saturations);
        }

        private double computeSumW(int index)
        {
            int height = nCols + 1;
            int width = nRows + 1;
            int i = index / height;
            int j = index % width;
            // compute Wik (k is the index of the neighboring cloth particle)
            int[] neighbors = new int[8];
            neighbors[0] = (i - 1) * width + (j - 1);
            neighbors[1] = (i - 1) * width + j;
            neighbors[2] = (i - 1) * width + (j + 1);
            neighbors[3] = i * width + (j - 1);
            neighbors[4] = i * width + (j + 1);
            neighbors[5] = (i + 1) * width + (j - 1);
            neighbors[6] = (i + 1) * width + j;
            neighbors[7] = (i + 1) * width + (j + 1);

            Vector3d pi = Particles[index].Predicted;

            double Wik = 0;
            for (int k = 0; k < 8; k++)
            {
                if (neighbors[k] >= 0 && neighbors[k] < NumParticles)
                {
                    Vector3d pk = Particles[neighbors[k]].Predicted;
                    Vector3d temp = new Vector3d(pi.x - pk.x, pi.y - pk.y, pi.z - pk.z);
                    Wik += ClothKernel.W(temp);
                }
            }
            return Wik;
        }

    }

    

}