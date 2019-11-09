using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Common.Geometry.Shapes;
using Common.Mathematics.LinearAlgebra;

    
using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Bodies.Fluids;
using PositionBasedDynamics.Collisions;

namespace PositionBasedDynamics.Solvers
{
    public class UnifiedSolver3d
    {
        //Matrix m_invM, m_JT, m_A;
        //double[] m_b, m_gamma, m_dp;
        //int[] m_counts;
        //int m_nParts, m_nCons;
        //LinearEquation m_eq;

        public UnifiedSolver3d()
        {

        }


        private const double RELAXATION_PARAMETER = 1.0;
    }

}
