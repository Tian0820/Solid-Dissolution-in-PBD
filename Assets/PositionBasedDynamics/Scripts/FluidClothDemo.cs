using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

using Common.Mathematics.LinearAlgebra;
using Common.Geometry.Shapes;
using Common.Unity.Drawing;
using Common.Unity.Mathematics;

using PositionBasedDynamics.Solvers;
using PositionBasedDynamics.Bodies;
using PositionBasedDynamics.Sources;
using PositionBasedDynamics.Forces;
using PositionBasedDynamics.Collisions;

namespace PositionBasedDynamics
{
    public class FluidClothDemo : MonoBehaviour
    {
        //[SerializeField]
        //Material m_SolidSphereMaterial;

        [SerializeField]
        Material m_BoundaryMaterial;

        [SerializeField]
        Button m_ResetButton;

        [SerializeField]
        Slider m_StrenchSlider;

        [SerializeField]
        Slider m_BendSlider;

        [SerializeField]
        Slider m_ClothSizeSlider;

        [SerializeField]
        Slider m_FluidSizeSlider;

        private const double timeStep = 1.0 / 60.0;

        public Material sphereMaterial;

        private List<GameObject> ClothSpheres { get; set; }

        private List<GameObject> FluidSpheres { get; set; }

        private GameObject[] BoundarySpheres { get; set; }

        private Body3d ClothBody { get; set; }

        private FluidBody3d FluidBody { get; set; }

        private ClothSolver3d ClothSolver { get; set; }

        private FluidSolver3d FluidSolver { get; set; }

        private List<Vector3d> StaticBounds { get; set; }

        private FluidBoundary3d Boundary { get; set; }

        private List<Box3d> FluidBounds;

        private double StretchStiffness;

        private double BendStiffness;

        private double ClothSize;

        private double FluidSize;

        private double ClothHeight = 3.0;

        private double FluidHeight = 3.5;

        void Start()
        {
            ClothSize = getSizeValue(m_ClothSizeSlider);
            FluidSize = getSizeValue(m_FluidSizeSlider);
            StretchStiffness = getStiffnessValue(m_StrenchSlider);
            BendStiffness = getStiffnessValue(m_BendSlider);

            InitializeFluid();
            InitializeCloth();

            AddEventListener();
        }

        void Update()
        {
            int iterations = 4;
            double dt = timeStep / iterations;

            for (int i = 0; i < iterations; i++)
            {
                ClothSolver.StepPhysics(dt);
            }

            FluidSolver.StepPhysics(timeStep);
            UpdateFluidSpheres();
            UpdateClothSpheres();

        }

        void OnDestroy()
        {
            if (ClothSpheres != null)
            {
                for (int i = 0; i < ClothSpheres.Count; i++)
                    DestroyImmediate(ClothSpheres[i]);
            }

            if (FluidSpheres != null)
            {
                for (int i = 0; i < FluidSpheres.Count; i++)
                {
                    DestroyImmediate(FluidSpheres[i]);
                    FluidSpheres[i] = null;
                }
            }

            if (BoundarySpheres != null)
            {
                for (int i = 0; i < BoundarySpheres.Length; i++)
                {
                    DestroyImmediate(BoundarySpheres[i]);
                    BoundarySpheres[i] = null;
                }
            }
        }

        public void InitializeFluid()
        {
            double radius = 0.1;
            double density = 1000.0;

            CreateBoundary(radius, density);
            CreateFluid(radius, density);

            FluidSolver = new FluidSolver3d(FluidBody);
            FluidSolver.AddForce(new GravitationalForce3d());
        }

        public void InitializeCloth()
        {
            double mass = 1.0;
            double radius = 0.1;

            double width = ClothSize;
            double height = ClothHeight;
            double depth = ClothSize;

            TrianglesFromGrid source = new TrianglesFromGrid(radius, width, depth);

            Matrix4x4d T = Matrix4x4d.Translate(new Vector3d(0.0, height, 0.0));
            Matrix4x4d R = Matrix4x4d.Rotate(new Vector3d(0.0, 0.0, 0.0));
            Matrix4x4d RT = T * R;

            //Body = new Body3d(source, radius, mass, stretchStiffness, bendStiffness, RT);
            ClothBody = new ClothBody3d(ParticlePhase.CLOTH, source, radius, mass, StretchStiffness, BendStiffness, RT);
            ClothBody.Dampning = 1.0;

            //Vector3d min = new Vector3d(-width / 2 - 0.1, height - 0.1, -depth / 2 - 0.1);
            //Vector3d max = new Vector3d(width / 2 + 0.1, height + 0.1, -depth / 2 + 0.1);
            Vector3d upright = new Vector3d(-width / 2, height, -depth / 2);
            Vector3d upleft = new Vector3d(width / 2, height, -depth / 2);
            Vector3d downright = new Vector3d(-width / 2, height, depth / 2);
            Vector3d downleft = new Vector3d(width / 2, height, depth / 2);

            StaticBounds = new List<Vector3d>();
            StaticBounds.Add(upright);
            StaticBounds.Add(upleft);
            StaticBounds.Add(downright);
            StaticBounds.Add(downleft);

            ClothBody.MarkAsStatic(StaticBounds);

            ClothSolver = new ClothSolver3d();
            ClothSolver.AddBody(ClothBody);
            ClothSolver.FluidBody = FluidBody;
            ClothSolver.AddForce(new GravitationalForce3d());
            ClothSolver.AddCollision(new ParticleCollision3d(0));
            ClothSolver.AddCollision(new PlanarCollision3d(Vector3d.UnitY, 0));
            ClothSolver.SolverIterations = 2;
            ClothSolver.CollisionIterations = 2;
            ClothSolver.SleepThreshold = 1;

            CreateCloth();
        }

        public void CreateFluid(double radius, double density)
        {
            //To make less particles decrease the size of the bounds or increase the radius.
            //Make sure fluid bounds fits inside the boundrys bounds.

            FluidBounds = new List<Box3d>();
            FluidBounds.Add(new Box3d(-0.5, 0.5, FluidHeight, FluidHeight + FluidSize, -0.5, 0.5));

            ParticlesFromBounds source = new ParticlesFromBounds(radius, FluidBounds[0]);

            System.Random rnd = new System.Random(0);
            FluidBody3d body = new FluidBody3d(ParticlePhase.FLUID, source, radius, density, Matrix4x4d.Identity);

            FluidBody = body;

            FluidBody.Dampning = 0.0;
            FluidBody.AddBoundary(Boundary);
            FluidBody.RandomizePositions(rnd, radius * 0.01);
            FluidBody.RandomizePositionOrder(rnd);

            FluidSpheres = new List<GameObject>();

            float diam = (float)FluidBody.Particles[0].ParticleDiameter;

            Debug.Log(FluidBody.Particles.Count);

            for (int i = 0; i < FluidBody.Particles.Count; i++)
            {
                Vector3d pos = FluidBody.Particles[i].Position;

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.parent = transform;
                sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;

                //if (View.Equals(ViewType.NORMAL))
                //    sphere.GetComponent<MeshRenderer>().material = m_FluidSphereMaterial_Standard;
                //else
                //    sphere.GetComponent<MeshRenderer>().material = m_FluidSphereMaterial_Transparent;

                FluidSpheres.Add(sphere);
            }

        }

        private void CreateCloth()
        {
            if (sphereMaterial == null) return;

            ClothSpheres = new List<GameObject>();

            int numParticles = ClothBody.NumParticles;
            //float diam = (float)Body.ParticleRadius * 2.0f;

            float diam = (float)ClothBody.Particles[0].ParticleDiameter;

            for (int i = 0; i < numParticles; i++)
            {
                //Vector3 pos = MathConverter.ToVector3(Body.Positions[i]);
                Vector3 pos = MathConverter.ToVector3(ClothBody.Particles[i].Position);

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.parent = transform;
                sphere.transform.position = pos;
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;
                sphere.GetComponent<MeshRenderer>().material = sphereMaterial;
                ClothSpheres.Add(sphere);

                Color originColor = sphereMaterial.color;
                ClothBody.Particles[i].Color = new Vector4d(originColor.r, originColor.g, originColor.b, originColor.a);
            }
            //Debug.Log("origin color: " + ClothBody.Particles[0].Color);
        }

        public void UpdateClothSpheres()
        {
            if (ClothSpheres != null)
            {
                for (int i = 0; i < ClothSpheres.Count; i++)
                {
                    //Vector3d pos = Body.Positions[i];
                    Vector3d pos = ClothBody.Particles[i].Position;
                    //Debug.Log("create" + pos);

                    ClothSpheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                    Vector4d color = ClothBody.Particles[i].Color;
                    //Debug.Log("update color " + i + ": " + ClothBody.Particles[i].Color);
                    ClothSpheres[i].GetComponent<MeshRenderer>().material.color = new Color((float)color.x, (float)color.y, (float)color.z, (float)color.w);
                }
            }

        }

        public void UpdateFluidSpheres()
        {
            if (FluidSpheres != null)
            {
                if (FluidSpheres.Count != FluidSolver.Body.NumParticles)
                {
                    //Debug.Log("Transform: " + FluidSpheres.Count + ", " + FluidSolver.Body.NumParticles);
                    for (int i = FluidSpheres.Count; i < FluidSolver.Body.NumParticles; i++)
                    {
                        Particle addParticle = FluidSolver.Body.Particles[i];
                        float diam = (float)addParticle.ParticleDiameter;
                        Vector3d pos = addParticle.Position;
                        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                        sphere.transform.parent = transform;
                        sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                        sphere.transform.localScale = new Vector3(diam, diam, diam);
                        sphere.GetComponent<Collider>().enabled = false;
                        //sphere.GetComponent<MeshRenderer>().material = m_FluidSphereMaterial_Standard;
                        sphere.GetComponent<MeshRenderer>().material.SetColor("_Color", Color.red);
                        FluidSpheres.Add(sphere);
                    }
                }

                for (int i = 0; i < FluidSolver.Body.NumParticles; i++)
                {
                    //Debug.Log("Body positons: " + Body.Positions[i]);
                    Vector3d pos = FluidSolver.Body.Particles[i].Position;
                    //Debug.Log(FluidSolver.Body.Particles[i].Position);
                    FluidSpheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                }
            }

        }

        public void CreateBoundary(double radius, double density)
        {

            Box3d InnerBounds = new Box3d(-3, 3, 0, 8, -3, 3);
            Box3d OuterBounds = InnerBounds;

            int thickness = 1;
            OuterBounds.Min -= radius * 2 * thickness;
            OuterBounds.Max += radius * 2 * thickness;

            ParticleSource source = new ParticlesFromBounds(radius, OuterBounds, InnerBounds);

            Boundary = new FluidBoundary3d(source, radius, density, Matrix4x4d.Identity);

            BoundarySpheres = new GameObject[Boundary.NumParticles];
            float diam = (float)Boundary.Particles[0].ParticleDiameter;

            for (int i = 0; i < BoundarySpheres.Length; i++)
            {
                Vector3d pos = Boundary.Particles[i].Position;

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);

                sphere.SetActive(false);
                sphere.transform.parent = transform;
                sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;

                sphere.GetComponent<MeshRenderer>().material = m_BoundaryMaterial;

                BoundarySpheres[i] = sphere;
            }

        }

        private void AddEventListener()
        {
            m_ResetButton.onClick.AddListener(delegate
            {
                ResetButtonClicked();
            });

            m_ClothSizeSlider.onValueChanged.AddListener(delegate
            {
                ClothSizeReset(m_ClothSizeSlider);
            });

            m_FluidSizeSlider.onValueChanged.AddListener(delegate
            {
                FluidSizeReset(m_FluidSizeSlider);
            });

            m_StrenchSlider.onValueChanged.AddListener(delegate
            {
                StrenchStiffnessReset(m_StrenchSlider);
            });

            m_BendSlider.onValueChanged.AddListener(delegate
            {
                BendStiffnessReset(m_BendSlider);
            });
        }

        private void ResetButtonClicked()
        {
            OnDestroy();
            Start();
        }

        private void ClothSizeReset(Slider slider)
        {
            OnDestroy();
            ClothSize = slider.value * 0.5;
            Start();
        }

        private void FluidSizeReset(Slider slider)
        {
            OnDestroy();
            FluidSize = slider.value * 0.5;
            Start();
        }

        private void StrenchStiffnessReset(Slider slider)
        {
            OnDestroy();
            StretchStiffness = slider.value;
            Start();
        }

        private void BendStiffnessReset(Slider slider)
        {
            OnDestroy();
            BendStiffness = slider.value;
            Start();
        }

        private double getSizeValue(Slider slider)
        {
            return slider.value * 0.5;
        }

        private double getStiffnessValue(Slider slider)
        {
            return slider.value;
        }
    }
}
