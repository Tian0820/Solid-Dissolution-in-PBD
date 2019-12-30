using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

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
        private const double timeStep = 1.0 / 60.0;

        private const int GRID_SIZE = 10;

        public Material fluidSphereMaterial;

        public Material solidSphereMaterial;

        public Material boundaryMaterial;

        public bool drawLines = true;

        public bool drawBoundary = false;

        private GameObject[] FluidSpheres { get; set; }

        private GameObject[] BoundarySpheres { get; set; }

        private Body3d FluidBody { get; set; }

        private Body3d SolidBody { get; set; }

        private FluidBoundary3d Boundary { get; set; }

        private Box3d OuterBounds, InnerBounds;

        private List<Box3d> FluidBounds;

        private SolidSolver3d SolidSolver { get; set; }

        private FluidSolver3d FluidSolver { get; set; }

        private List<Dictionary<Particle, GameObject>> SolidSpheres { get; set; }

        // Start is called before the first frame update
        void Start()
        {
            InitializeFluid();
            InitializeSolid();
        }

        // Update is called once per frame
        void Update()
        {
            FluidSolver.StepPhysics(timeStep);

            int iterations = 4;
            double dt = timeStep / iterations;

            for (int i = 0; i < iterations; i++)
                SolidSolver.StepPhysics(dt);

            UpdateFluidSpheres();
            UpdateSolidSpheres();
        }

        void OnDestroy()
        {

            if (FluidSpheres != null)
            {
                for (int i = 0; i < FluidSpheres.Length; i++)
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

        private void OnRenderObject()
        {
            Camera camera = Camera.current;

            Vector3 min = new Vector3(-GRID_SIZE, 0, -GRID_SIZE);
            Vector3 max = new Vector3(GRID_SIZE, 0, GRID_SIZE);

            Matrix4x4d m = MathConverter.ToMatrix4x4d(transform.localToWorldMatrix);
            DrawLines.DrawBounds(camera, Color.red, OuterBounds, m);
            DrawLines.DrawBounds(camera, Color.red, InnerBounds, m);
            for (int i = 0; i < FluidBounds.Count; i++)
            {
                DrawLines.DrawBounds(camera, Color.blue, FluidBounds[i], m);
            }
            DrawLines.DrawGrid(camera, Color.white, min, max, 1, transform.localToWorldMatrix);

        }

        public void InitializeFluid()
        {
            double radius = 0.25;
            double density = 1000.0;

            CreateBoundary(radius, density);
            CreateFluid(radius, density);

            FluidSolver = new FluidSolver3d(FluidBody);
            FluidSolver.AddForce(new GravitationalForce3d());

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

            T = Matrix4x4d.Translate(new Vector3d(0.0, 10.0, 0.0));
            R = Matrix4x4d.Rotate(new Vector3d(0.0, 0.0, 25.0));

            Body3d body = new Body3d(ParticlePhase.SOLID, source, radius, mass, T * R);
            body.Dampning = 1.0;
            body.RandomizePositions(rnd, radius * 0.01);
            SolidBody = body;

            SolidSolver = new SolidSolver3d();
            SolidSolver.AddBody(SolidBody);
            SolidSolver.FluidBody = FluidBody;

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

            InnerBounds = new Box3d(-3, 3, 0, 8, -3, 3);
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

                sphere.GetComponent<MeshRenderer>().material = boundaryMaterial;

                BoundarySpheres[i] = sphere;
            }

        }

        public void CreateFluid(double radius, double density)
        {
            //To make less particles decrease the size of the bounds or increase the radius.
            //Make sure fluid bounds fits inside the boundrys bounds.

            //FluidBounds = new Box3d(-3, 3, 0, 5, -3, 3);
            FluidBounds = new List<Box3d>();
            FluidBounds.Add(new Box3d(-1, 1, 4, 6, -1, 1));
            FluidBounds.Add(new Box3d(-3, 3, 0, 3, -3, 3));

            ParticlesFromBounds source1 = new ParticlesFromBounds(radius, FluidBounds[0]);
            ParticlesFromBounds source2 = new ParticlesFromBounds(radius, FluidBounds[1]);

            System.Random rnd = new System.Random(0);

            //Body3d body1 = new Body3d(ParticlePhase.FLUID, source1, radius, density, Matrix4x4d.Identity);
            Body3d body2 = new Body3d(ParticlePhase.FLUID, source2, radius, density, Matrix4x4d.Identity);

            //Body = body1.ContactBody3d(body2);
            FluidBody = body2;

            FluidBody.Dampning = 0.0;
            FluidBody.AddBoundry(Boundary);
            FluidBody.RandomizePositions(rnd, radius * 0.01);
            FluidBody.RandomizePositionOrder(rnd);

            FluidSpheres = new GameObject[FluidBody.NumParticles];

            float diam = (float)FluidBody.Particles[0].ParticleDiameter;

            for (int i = 0; i < FluidSpheres.Length; i++)
            {
                Vector3d pos = FluidBody.Particles[i].Position;

                GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                sphere.transform.parent = transform;
                sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                sphere.transform.localScale = new Vector3(diam, diam, diam);
                sphere.GetComponent<Collider>().enabled = false;

                sphere.GetComponent<MeshRenderer>().material = fluidSphereMaterial;

                FluidSpheres[i] = sphere;
            }

        }

        private void CreateSolidSpheres()
        {
            if (solidSphereMaterial == null) return;

            SolidSpheres = new List<Dictionary<Particle, GameObject>>();

            for (int j = 0; j < SolidSolver.SolidBodies.Count; j++)
            {
                Body3d body = SolidSolver.SolidBodies[j];

                int numParticles = body.NumParticles;

                //Debug.Log("solid body.NumParticles: " + numParticles);

                float diam = (float)body.Particles[0].ParticleDiameter;

                Dictionary<Particle, GameObject> spheres = new Dictionary<Particle, GameObject>(numParticles);

                for (int i = 0; i < numParticles; i++)
                {
                    Vector3 pos = MathConverter.ToVector3(body.Particles[i].Position);

                    GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    sphere.transform.parent = transform;
                    sphere.transform.position = pos;
                    sphere.transform.localScale = new Vector3(diam, diam, diam);
                    sphere.GetComponent<Collider>().enabled = false;
                    sphere.GetComponent<MeshRenderer>().material = solidSphereMaterial;
                    spheres[body.Particles[i]] = sphere;
                }
                SolidSpheres.Add(spheres);
            }
        }

        public void UpdateFluidSpheres()
        {
            if (FluidSpheres != null)
            {
                for (int i = 0; i < FluidSpheres.Length; i++)
                {
                    //Debug.Log("Body positons: " + Body.Positions[i]);
                    Vector3d pos = FluidBody.Particles[i].Position;
                    FluidSpheres[i].transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
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
                    Dictionary<Particle, GameObject> spheres = SolidSpheres[j];

                    for (int i = 0; i < spheres.Count; i++)
                    {
                        Vector3d pos = body.Particles[i].Position;
                        GameObject sphere = spheres[body.Particles[i]];
                        sphere.transform.position = new Vector3((float)pos.x, (float)pos.y, (float)pos.z);
                        if (body.Particles[i].needTrans == true)
                        {
                            sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
                        }
                    }
                }
            }
        }

    }
}
