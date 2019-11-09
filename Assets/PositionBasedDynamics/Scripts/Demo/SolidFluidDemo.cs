using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;
using Common.Unity.Drawing;
using Common.Unity.Mathematics;

using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Bodies.Fluids;
using PositionBasedDynamics.Constraints;
using PositionBasedDynamics.Solvers;
using PositionBasedDynamics.Sources;

namespace PositionBasedDynamics
{
    public class SolidFluidDemo : MonoBehaviour
    {
        List<Particle> m_Particles { get; set; }
        List<UnifiedBody3d> m_Bodies { get; set; }
        Dictionary<Particle.ConstraintGroup, List<UnifiedConstraint3d>> m_GlobalConstraints { get; set; }
        UnifiedSolver3d m_StandardSolver { get; set; }
        UnifiedSolver3d m_ContactSolver { get; set; }

        FluidBoundary3d Boundary { get; set; }
        private Box3d FluidBounds, OuterBounds, InnerBounds;
        GameObject[] BoundarySpheres { get; set; }
        GameObject[] FluidSpheres { get; set; }
        List<List<GameObject>> SolidSpheres { get; set; }

        Vector3d m_Gravity;
        Vector2d m_XBoundaries;
        Vector2d m_YBoundaries;
        Vector2d m_ZBoundaries;

        public Material fluidSphereMaterial;
        public Material solidSphereMaterial;
        public Material boundaryMaterial;


        // Start is called before the first frame update
        void Start()
        {
            InitialFluidSolid();
        }

        void InitialFluidSolid()
        {
            double scale = 0.3, delta = 0.7;
            m_XBoundaries = new Vector2d(-3, 3);
            m_YBoundaries = new Vector2d(0, 8);
            m_ZBoundaries = new Vector2d(-3, 3);
            List<Particle> particles = new List<Particle>();

            double num = 0.1f;
            for (int d = 0; d < num; d++)
            {
                double start = -2 * scale + 4 * scale * (d / num);
                for (double x = start; x < start + (4 * scale / num); x += delta)
                {
                    for (double y = -2 * scale; y < 2 * scale; y += delta)
                    {
                        for (double z = -2 * scale; z < 2 * scale; z += delta)
                        {
                            Particle temp = new Particle(new Vector3d(x, y + 3, z) + 0.2f * new Vector3d(frand() - 0.5, frand() - 0.5, frand() - 0.5), 1);
                            particles.Add(temp);
                        }
                    }
                }
                CreateFluid(particles, 1 + 1.25 * (d + 1));
                particles.Clear();
            }

        }

        // Update is called once per frame
        void Update()
        {

        }

        TotalFluidConstraint3d CreateFluid(List<Particle> verts, double density)
        {
            int offset = m_Particles.Count;
            int bod = (int)(100 * frand());
            List<int> indices = new List<int>();
            for (int i = 0; i < verts.Count; i++)
            {
                Particle p = verts[i];
                p.phase = Particle.Phase.FLUID;
                p.Bod = bod;

                if (p.Imass == 0)
                {
                    Debug.Log("A fluid cannot have a point of infinite mass.");
                }

                m_Particles.Add(p);
                indices.Add(offset + i);
            }
            TotalFluidConstraint3d fs = new TotalFluidConstraint3d(density, indices);
            m_GlobalConstraints[Particle.ConstraintGroup.STANDARD].Add(fs);
            return fs;
        }

        public void CreateBoundary(double radius, double density)
        {

            InnerBounds = new Box3d(-3, 3, 0, 8, -3, 3);
            OuterBounds = InnerBounds;

            int thickness = 1;
            OuterBounds.Min -= new Vector3d(radius * 2 * thickness);
            OuterBounds.Max += new Vector3d(radius * 2 * thickness);

            ParticleSource source = new ParticlesFromBounds(radius, OuterBounds, InnerBounds);

            Boundary = new FluidBoundary3d(source, radius, density, Matrix4x4d.Identity);

            BoundarySpheres = new GameObject[Boundary.NumParticles];
            float diam = (float)Boundary.ParticleDiameter;

            for (int i = 0; i < BoundarySpheres.Length; i++)
            {
                Vector3d pos = Boundary.Positions[i];

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);

                //sphere.SetActive(drawBoundary);
                sphere.transform.parent = transform;
                sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;

                sphere.GetComponent<MeshRenderer>().material = boundaryMaterial;

                BoundarySpheres[i] = sphere;
            }

        }

        private double frand()
        {
            return Random.value / double.MaxValue;
        }
    }
}
