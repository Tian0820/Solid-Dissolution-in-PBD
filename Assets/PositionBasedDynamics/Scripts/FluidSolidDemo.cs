﻿using System;
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
    public class FluidSolidDemo : MonoBehaviour
    {
        [SerializeField]
        Dropdown m_RateDropdown;

        [SerializeField]
        Dropdown m_ViewDropdown;

        [SerializeField]
        Material m_FluidSphereMaterial_Standard;

        [SerializeField]
        Material m_FluidSphereMaterial_Transparent;

        [SerializeField]
        Material m_SolidSphereMaterial;

        [SerializeField]
        Material m_BoundaryMaterial;

        private const double timeStep = 1.0 / 60.0;

        private const int GRID_SIZE = 10;

        public bool drawLines = true;

        public bool drawBoundary = false;

        public int DissolutionRate = 10;

        private List<GameObject> FluidSpheres { get; set; }

        private GameObject[] BoundarySpheres { get; set; }

        private FluidBody3d FluidBody { get; set; }

        //private Body3d SolidBody { get; set; }

        private FluidBoundary3d Boundary { get; set; }

        private Box3d OuterBounds, InnerBounds;

        private List<Box3d> FluidBounds;

        private SolidSolver3d SolidSolver { get; set; }

        private FluidSolver3d FluidSolver { get; set; }

        private List<List<GameObject>> SolidSpheres { get; set; }

        private ViewType View = ViewType.NORMAL;

        enum ViewType
        {
            NORMAL,
            CLEAR
        }

        // Start is called before the first frame update
        void Start()
        {
            InitializeFluid();
            InitializeSolid();

            m_RateDropdown.onValueChanged.AddListener(delegate {
                RateDropdownValueChanged(m_RateDropdown);
            });

            m_ViewDropdown.onValueChanged.AddListener(delegate {
                ViewDropdownValueChanged(m_ViewDropdown);
            });
        }

        // Update is called once per frame
        void Update()
        {
            int iterations = 4;
            double dt = timeStep / iterations;

            for (int i = 0; i < iterations && SolidExist(); i++)
            {
                SolidSolver.StepPhysics(dt);
                FluidSolver.UpdateParticleToTrans(SolidSolver.ParticleToTrans);
                SolidSolver.FluidBody = FluidSolver.Body;
            }

            FluidSolver.StepPhysics(timeStep);

            UpdateSolidSpheres();
            UpdateFluidSpheres();
        }

        void OnDestroy()
        {

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

            if (SolidSpheres != null)
            {
                for (int i = 0; i < SolidSpheres.Count; i++)
                {
                    for (int j = 0; j < SolidSpheres[i].Count; j++)
                    {
                        DestroyImmediate(SolidSpheres[i][j]);
                        SolidSpheres[i][j] = null;
                    }
                }
            }

        }

        private void OnRenderObject()
        {
            Camera camera = Camera.current;

            Vector3 min = new Vector3(-GRID_SIZE, 0, -GRID_SIZE);
            Vector3 max = new Vector3(GRID_SIZE, 0, GRID_SIZE);

            Matrix4x4d m = MathConverter.ToMatrix4x4d(transform.localToWorldMatrix);
            DrawLines.DrawBounds(camera, Color.gray, OuterBounds, m);
            DrawLines.DrawBounds(camera, Color.gray, InnerBounds, m);
            //for (int i = 0; i < FluidBounds.Count; i++)
            //{
            //    DrawLines.DrawBounds(camera, Color.blue, FluidBounds[i], m);
            //}
            //DrawLines.DrawGrid(camera, Color.white, min, max, 1, transform.localToWorldMatrix);

        }

        public void InitializeFluid()
        {
            double radius = 0.25;
            double density = 1000.0;

            CreateBoundary(radius, density);
            CreateFluid(radius, density);

            FluidSolver = new FluidSolver3d(FluidBody);
            FluidSolver.AddForce(new GravitationalForce3d());
            FluidSolver.DissolutionRate = DissolutionRate;

        }

        public void InitializeSolid()
        {
            Matrix4x4d T, R;

            double spacing = 0.125;
            double radius = spacing;
            double mass = 1.0;
            System.Random rnd = new System.Random(0);

            Vector3d min = new Vector3d(-0.5);
            Vector3d max = new Vector3d(0.5);
            Box3d bounds = new Box3d(min, max);

            ParticlesFromBounds source = new ParticlesFromBounds(spacing, bounds);

            T = Matrix4x4d.Translate(new Vector3d(0.0, 10.0, -1.0));
            R = Matrix4x4d.Rotate(new Vector3d(0.0, 0.0, 25.0));

            Body3d body = new SolidBody3d(ParticlePhase.SOLID, source, radius, mass, T * R);
            body.Dampning = 1.0;
            body.RandomizePositions(rnd, radius * 0.01);

            SolidSolver = new SolidSolver3d();
            SolidSolver.AddBody(body);
            SolidSolver.FluidBody = FluidBody;
            SolidSolver.DissolutionRate = DissolutionRate;

            SolidSolver.AddForce(new GravitationalForce3d());
            SolidSolver.AddCollision(new ParticleCollision3d(0));
            SolidSolver.AddCollision(new PlanarCollision3d(Vector3d.UnitY, 0));
            SolidSolver.SolverIterations = 2;
            SolidSolver.CollisionIterations = 2;
            SolidSolver.SleepThreshold = 1;

            CreateSolidSpheres();
        }

        public void CreateBoundary(double radius, double density)
        {

            InnerBounds = new Box3d(-2, 2, 0, 8, -2, 2);
            OuterBounds = InnerBounds;

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

                sphere.SetActive(drawBoundary);
                sphere.transform.parent = transform;
                sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;

                sphere.GetComponent<MeshRenderer>().material = m_BoundaryMaterial;

                BoundarySpheres[i] = sphere;
            }

        }

        public void CreateFluid(double radius, double density)
        {
            //To make less particles decrease the size of the bounds or increase the radius.
            //Make sure fluid bounds fits inside the boundrys bounds.

            //FluidBounds = new Box3d(-3, 3, 0, 5, -3, 3);
            FluidBounds = new List<Box3d>();
            //FluidBounds.Add(new Box3d(-1, 1, 4, 6, -1, 1));
            FluidBounds.Add(new Box3d(-2, 2, 0, 4, -2, 2));

            //ParticlesFromBounds source1 = new ParticlesFromBounds(radius, FluidBounds[0]);
            ParticlesFromBounds source2 = new ParticlesFromBounds(radius, FluidBounds[0]);

            System.Random rnd = new System.Random(0);

            //Body3d body1 = new Body3d(ParticlePhase.FLUID, source1, radius, density, Matrix4x4d.Identity);
            FluidBody3d body2 = new FluidBody3d(ParticlePhase.FLUID, source2, radius, density, Matrix4x4d.Identity);

            //Body = body1.ContactBody3d(body2);
            FluidBody = body2;

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

                if (View.Equals(ViewType.NORMAL))
                    sphere.GetComponent<MeshRenderer>().material = m_FluidSphereMaterial_Standard;
                else
                    sphere.GetComponent<MeshRenderer>().material = m_FluidSphereMaterial_Transparent;

                FluidSpheres.Add(sphere);
            }

        }

        private void CreateSolidSpheres()
        {
            if (m_SolidSphereMaterial == null) return;

            SolidSpheres = new List<List<GameObject>>();

            for (int j = 0; j < SolidSolver.SolidBodies.Count; j++)
            {
                Body3d body = SolidSolver.SolidBodies[j];

                int numParticles = body.NumParticles;

                //Debug.Log("solid body.NumParticles: " + numParticles);

                float diam = (float)body.Particles[0].ParticleDiameter;

                List<GameObject> spheres = new List<GameObject>(numParticles);

                for (int i = 0; i < numParticles; i++)
                {
                    Vector3 pos = MathConverter.ToVector3(body.Particles[i].Position);

                    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    sphere.transform.parent = transform;
                    sphere.transform.position = pos;
                    sphere.transform.localScale = new Vector3(diam, diam, diam);
                    sphere.GetComponent<Collider>().enabled = false;
                    sphere.GetComponent<MeshRenderer>().material = m_SolidSphereMaterial;
                    spheres.Add(sphere);
                }
                SolidSpheres.Add(spheres);
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
                        sphere.GetComponent<MeshRenderer>().material = m_FluidSphereMaterial_Standard;
                        sphere.GetComponent<MeshRenderer>().material.SetColor("_Color", Color.red);
                        FluidSpheres.Add(sphere);
                    }
                }

                for (int i = 0; i < FluidSolver.Body.NumParticles; i++)
                {
                    //Debug.Log("Body positons: " + Body.Positions[i]);
                    Vector3d pos = FluidSolver.Body.Particles[i].Position;
                    Vector4d color = FluidSolver.Body.Particles[i].Color;
                    //Debug.Log(FluidSolver.Body.Particles[i].Position);
                    FluidSpheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                    if (View.Equals(ViewType.NORMAL))
                        FluidSpheres[i].GetComponent<MeshRenderer>().material.color = new Color((float)color.x, (float)color.y, (float)color.z, (float)color.w);
                }
            }

            if (BoundarySpheres != null)
            {
                for (int i = 0; i < BoundarySpheres.Length; i++)
                {
                    BoundarySpheres[i].SetActive(drawBoundary);

                    Vector3d pos = Boundary.Particles[i].Position;
                    BoundarySpheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                }
            }
        }

        public void UpdateSolidSpheres()
        {
            if (SolidSpheres != null)
            {
                for (int j = 0; j < SolidSolver.SolidBodies.Count; j++)
                {
                    Body3d body = SolidSolver.SolidBodies[j];
                    List<GameObject> spheres = SolidSpheres[j];
                    List<int> particleIndex = new List<int>();
                    int originCount = spheres.Count;

                    for (int i = 0; i < body.Particles.Count; i++)
                    {
                        particleIndex.Add(body.Particles[i].Index);
                    }

                    for (int i = 0; i < originCount; i++)
                    {
                        if (!particleIndex.Contains(i))
                        {
                            //if (i <= 1)
                            //    break;
                            //sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                            //Debug.Log("not contain particle no. " + i);
                            spheres[i].SetActive(false);
                        }
                    }

                    //Debug.Log("sphere count: " + spheres.Count + "; " + body.Particles.Count);

                    for (int i = 0; i < body.Particles.Count; i++)
                    {
                        //Debug.Log("particleIndex. " + particleIndex[i]);
                        GameObject sphere = spheres[particleIndex[i]];
                        Vector3d pos = body.Particles[i].Position;
                        sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                    }
                }
            }
        }

        private bool SolidExist()
        {
            bool exist = true;
            for (int i = 0; i < SolidSolver.SolidBodies.Count; i++)
            {
                if (SolidSolver.SolidBodies[i].NumParticles == 0)
                {
                    exist = false;
                }
            }
            return exist;
        }

        private void RateDropdownValueChanged(Dropdown change)
        {
            if (change.value == 0)
            {
                // Medium
                DissolutionRate = 10;
            } else if (change.value == 1)
            {
                // Fast
                DissolutionRate = 5;
            } else if (change.value == 2)
            {
                // Slow
                DissolutionRate = 15;
            }
            OnDestroy();
            Start();
        }

        private void ViewDropdownValueChanged(Dropdown change)
        {
            if (change.value == 0)
            {
                // Normal View
                View = ViewType.NORMAL;
            }
            else if (change.value == 1)
            {
                // Clear View
                View = ViewType.CLEAR;
            }
            OnDestroy();
            Start();
        }
    }
}
