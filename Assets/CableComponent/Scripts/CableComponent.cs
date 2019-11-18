using UnityEngine;
using System;
using System.Collections;


public class CableComponent : MonoBehaviour
{
    #region Class members

    [SerializeField] private Transform endPoint;
    [SerializeField] private Material cableMaterial;

    // Cable config
    public float cableLength = 0.5f;
    [SerializeField] private int totalSegments = 5;
    [SerializeField] private float segmentsPerUnit = 2f;
    private int segments = 0;
    [SerializeField] private float cableWidth = 0.1f;

    // Solver config
    [SerializeField] private int verletIterations = 1;
    [SerializeField] private int solverIterations = 1;

    //[Range(0,3)]
    [SerializeField] private float stiffness = 1f;

    public bool Collision = true;
    public float friction = 0.2f;

    private LineRenderer line;
    private CableParticle[] points;

    #endregion

Transform MyTransform;
    #region Initial setup
    public bool init;
    private void OnValidate()
    {
        if (init)
        {
            InitCableParticles();
            init = false;
            line.SetVertexCount(segments + 1);
        }

    }
    void Start()
    {
        // if(!GetComponent<Rigidbody>())gameObject.AddComponent<Rigidbody>();
        // if(!endPoint.GetComponent<Rigidbody>())endPoint.gameObject.AddComponent<Rigidbody>();
        // if(!GetComponent<SpringJoint>())
        MyTransform=transform;
        InitCableParticles();
        InitLineRenderer();
    }

    /**
	 * Init cable particles
	 * 
	 * Creates the cable particles along the cable length
	 * and binds the start and end tips to their respective game objects.
	 */
     public void  ResetCable(float t=-1){
        InitCableParticles();
        // InitLineRenderer();
        staticCableTimeout=t;
     }
    void InitCableParticles()
    {
        // Calculate segments to use
        if (totalSegments > 0)
            segments = totalSegments;
        else
            segments = Mathf.CeilToInt(cableLength * segmentsPerUnit);
        Vector3 cableDirection = Vector3.down;
        if (endPoint) cableDirection = (endPoint.position - MyTransform.position).normalized;
        float initialSegmentLength = cableLength / segments;
        points = new CableParticle[segments + 1];

        // Foreach point
        for (int pointIdx = 0; pointIdx <= segments; pointIdx++)
        {
            // Initial position
            Vector3 initialPosition = MyTransform.position + (cableDirection * (initialSegmentLength * pointIdx));
            points[pointIdx] = new CableParticle(initialPosition);
        }

        // Bind start and end particles with their respective gameobjects
        CableParticle start = points[0];
        CableParticle end = points[segments];
        start.Bind(this.transform);
        start.Bind(this.transform);
        if (endPoint) end.Bind(endPoint.transform);
    }

    /**
	 * Initialized the line renderer
	 */
    void InitLineRenderer()
    {
        line = this.gameObject.AddComponent<LineRenderer>();
        line.SetWidth(cableWidth, cableWidth);
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.TwoSided;
        line.SetVertexCount(segments + 1);
        line.material = cableMaterial;
        line.GetComponent<Renderer>().enabled = true;
    }

    #endregion


    #region Render Pass
    public float staticCableTimeout=-1; //-1 is no timeout
    float CalculateTimer = 0.01f;
    void LateUpdate()
    {
        if(staticCableTimeout==-1||staticCableTimeout>0){
        if (cableLength <= 0.05f)
        {
            cableLength = 0.05f;
            CalculateTimer = 0;
            var p=transform.position;
            for (int pointIdx = 0; pointIdx < segments + 1; pointIdx++)
            {
                    line.SetPosition(pointIdx, p);
            }
            VerletReset(p);

        }
        else
        {
            while (CalculateTimer > 0)
            {
                for (int verletIdx = 0; verletIdx < verletIterations; verletIdx++)
                {
                    VerletIntegrate(0.01f);
                    SolveConstraints();
                }
                CalculateTimer -= 0.01f;
            }
            RenderCable();
            CalculateTimer += Time.deltaTime;
        }
        if(staticCableTimeout>0)staticCableTimeout-=Time.deltaTime;
        }
        
    }

    /**
     * Render Cable
     * 
     * Update every particle position in the line renderer.
     */
    void RenderCable()
    {
        if (line.positionCount != segments + 1) line.positionCount = segments + 1;
        for (int pointIdx = 0; pointIdx < segments + 1; pointIdx++)
        {
            if (pointIdx == 0 || (pointIdx == segments))
            {
                points[pointIdx].UpdateVerlet(Vector3.zero, Time.deltaTime);
                line.SetPosition(pointIdx, points[pointIdx].Position);
            }
            else
                line.SetPosition(pointIdx, points[pointIdx].Position);
        }
    }

    #endregion


    #region Verlet integration & solver pass

    void FixedUpdate()
    {
        if (cableLength < 0.01f) cableLength = 0.01f;
        // for (int verletIdx = 0; verletIdx < verletIterations; verletIdx++)
        // {
        //     VerletIntegrate(Time.fixedDeltaTime);
        //     SolveConstraints();
        // }
    }

    /**
     * Verler integration pass
     * 
     * In this step every particle updates its position and speed.
     */
    void VerletIntegrate(float dt)
    {
        Vector3 gravityDisplacement;
        gravityDisplacement = dt * dt * Physics.gravity;
        foreach (CableParticle particle in points)
        {
            particle.UpdateVerlet(gravityDisplacement, dt);
        }
    }
    void VerletReset(Vector3 p){
        foreach (CableParticle particle in points)
        {
            particle.UpdatePosition(p);
        }
    }

    /**
     * Constrains solver pass
     * 
     * In this step every constraint is addressed in sequence
     */
    void SolveConstraints()
    {
        // For each solver iteration..
        for (int iterationIdx = 0; iterationIdx < solverIterations; iterationIdx++)
        {
            SolveDistanceConstraint();
            SolveStiffnessConstraint();
            if (Collision) SolveColission();
        }
    }

    #endregion


    #region Solver Constraints

    /**
     * Distance constraint for each segment / pair of particles
     **/
    void SolveDistanceConstraint()
    {
        float segmentLength = cableLength / segments;
        for (int SegIdx = 0; SegIdx < segments; SegIdx++)
        {
            CableParticle particleA = points[SegIdx];
            CableParticle particleB = points[SegIdx + 1];

            // Solve for this pair of particles
            SolveDistanceConstraint(particleA, particleB, segmentLength);
        }
    }

    /**
     * Distance Constraint 
     * 
     * This is the main constrains that keeps the cable particles "tied" together.
     */
    void SolveDistanceConstraint(CableParticle particleA, CableParticle particleB, float segmentLength)
    {
        // Find current vector between particles
        Vector3 delta = particleB.Position - particleA.Position;
        // 
        float currentDistance = delta.magnitude;
        float errorFactor = (currentDistance - segmentLength) / currentDistance;

        // Only move free particles to satisfy constraints
        if (particleA.IsFree() && particleB.IsFree())
        {
            particleA.Position += errorFactor * 0.5f * delta;
            particleB.Position -= errorFactor * 0.5f * delta;
        }
        else if (particleA.IsFree())
        {
            particleA.Position += errorFactor * delta;
        }
        else if (particleB.IsFree())
        {
            particleB.Position -= errorFactor * delta;
        }
    }

    /**
     * Stiffness constraint
     **/
    void SolveColission()
    {
        for (int SegIdx = 0; SegIdx < segments + 1; SegIdx++)
        {
            CableParticle particleA = points[SegIdx];
            if (particleA.IsFree() && particleA.Position.y < (cableWidth))
            {
                particleA.Position += (cableWidth - particleA.Position.y) * Vector3.up - particleA.Velocity * friction;
            }
        }
    }
    void SolveStiffnessConstraint()
    {
        float distance = (points[0].Position - points[segments].Position).magnitude;
        if (distance > cableLength)
        {
            foreach (CableParticle particle in points)
            {
                SolveStiffnessConstraint(particle, distance);
            }
        }
    }

    /**
     * TODO: I'll implement this constraint to reinforce cable stiffness 
     * 
     * As the system has more particles, the verlet integration aproach 
     * may get way too loose cable simulation. This constraint is intended 
     * to reinforce the cable stiffness.
     * // throw new System.NotImplementedException ();
     **/
    void SolveStiffnessConstraint(CableParticle cableParticle, float distance)
    {


    }

    #endregion
}
