using UnityEngine;
using wizardscode.agent;
using wizardscode.ai;
using wizardscode.digitalpainting.agent;

public class RobotMovementController : BaseMovementController
{
    [Header("Pathfinding")]
    [SerializeField] protected Octree octree;
    [Tooltip("The maximum distance from the final target that will trigger a rebuild of the path.")]
    [SerializeField] protected float maxDistanceRebuildPath = 1;
    [Tooltip("The minimum time between path rebuilds, in seconds.")]
    [SerializeField] protected float minTimeBetweenRebuilds = 5f;
    [SerializeField] protected float pathPointRadius = 0.2f;

    private float timeToNextRebuild;
    protected Octree.PathRequest oldPath;
    protected Octree.PathRequest newPath;
    new protected Rigidbody rigidbody;
    new protected Collider collider;

    FlyingMovementBrain movementBrain;
    
    public Octree Octree
    {
        get { return octree; }
    }

    internal float PreferredFlightHeight
    {
        get { return (movementBrain.MovementController.maximumFlyHeight - movementBrain.MovementController.minimumFlyHeight) / 2; }
    }

    internal float MinimumFlightHeight
    {
        get { return movementBrain.MovementController.minimumFlyHeight; }
    }

    internal float MaximumFlightHeight
    {
        get { return movementBrain.MovementController.maximumFlyHeight; }
    }

    // Use this for initialization
    void Start()
    {
        collider = GetComponent<Collider>();
        rigidbody = GetComponent<Rigidbody>();
        octree = FindObjectOfType<Octree>();
        if (octree == null)
        {
            Debug.LogError("There is no `octree` component in your seen. Please add one so that Flying-Pathfinding can work.");
        }

        movementBrain = GetComponent<FlyingMovementBrain>();

        timeToNextRebuild = minTimeBetweenRebuilds;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (MovementBrain.Target == null)
        {
            return;
        }
        if ((newPath == null || !newPath.isCalculating) && Vector3.SqrMagnitude(MovementBrain.Target.transform.position - lastDestination) > maxDistanceRebuildPath && !octree.IsBuilding)
        {
            lastDestination = MovementBrain.Target.transform.position;

            oldPath = newPath;
            newPath = octree.GetPath(transform.position, lastDestination, this);
        }

        /*if (newPath != null && !newPath.isCalculating)
		{
			if (newPath.Path.Count > 0)
			{
				float distanceSoFar = 0;
				int lastPoint = newPath.Path.Count - 1;
				for (int i = lastPoint; i >= 1; i--)
				{
					distanceSoFar += Vector3.Distance(newPath.Path[i], newPath.Path[i - 1]);

					if (distanceSoFar <= minFollowDistance)
					{
						lastPoint = i;
					}
					else
					{
						break;
					}
				}
				if (lastPoint > 0)
				{
					newPath.Path.RemoveRange(lastPoint, newPath.Path.Count - lastPoint);
				}
			}
		}*/

        var curPath = Path;
        timeToNextRebuild -= Time.deltaTime;
        float distanceFromTarget = Vector3.Distance(transform.position, MovementBrain.Target.position);
        if (timeToNextRebuild < 0 && distanceFromTarget > maxDistanceRebuildPath)
        {
            oldPath = newPath;
            newPath = octree.GetPath(transform.position, lastDestination, this);
            timeToNextRebuild = minTimeBetweenRebuilds;
        }
        else if (curPath != null && !curPath.isCalculating && curPath.Path.Count > 0)
        {
            float distanceFromWaypoint = Vector3.Distance(transform.position, curPath.Path[0]);
            
            if (MovementBrain.MovementController.useRootMotion && MovementBrain.AgentController.MovementType != BaseAgentController.MovementStyle.Idle)
            {
                float speed = MovementBrain.Speed;
                if (distanceFromWaypoint > 5 * MovementBrain.MovementController.minReachDistance)
                {
                    if (speed <= MovementBrain.MovementController.maxSpeed)
                    {
                        speed += Time.deltaTime * MovementBrain.MovementController.Acceleration;
                    }
                }
                else if (distanceFromWaypoint > MovementBrain.MovementController.minReachDistance)
                
                {
                    if (speed >= MovementBrain.MovementController.maxSpeed * MovementBrain.MovementController.slowMovementFactor)
                    {
                        speed -= Time.deltaTime * MovementBrain.MovementController.Acceleration;
                    }
                    else
                    {
                        speed += Time.deltaTime * MovementBrain.MovementController.Acceleration;
                    }
                }
                MovementBrain.Speed = speed;
            }
            else
            {
                rigidbody.velocity += Vector3.ClampMagnitude(curPath.Path[0] - transform.position, 1) * Time.deltaTime * MovementBrain.MovementController.Acceleration;
            }
            float sqrMinReachDistance = MovementBrain.MovementController.minReachDistance * MovementBrain.MovementController.minReachDistance;

            Vector3 predictedPosition = rigidbody.position + rigidbody.velocity * Time.deltaTime;
            float shortestPathDistance = Vector3.SqrMagnitude(predictedPosition - curPath.Path[0]);
            int shortestPathPoint = 0;

            for (int i = 0; i < curPath.Path.Count; i++)
            {
                float sqrDistance = Vector3.SqrMagnitude(rigidbody.position - curPath.Path[i]);
                if (sqrDistance <= sqrMinReachDistance)
                {
                    if (i < curPath.Path.Count)
                    {
                        curPath.Path.RemoveRange(0, i + 1);
                    }
                    shortestPathPoint = 0;
                    break;
                }

                float sqrPredictedDistance = Vector3.SqrMagnitude(predictedPosition - curPath.Path[i]);
                if (sqrPredictedDistance < shortestPathDistance)
                {
                    shortestPathDistance = sqrPredictedDistance;
                    shortestPathPoint = i;
                }
            }

            if (shortestPathPoint > 0)
            {
                curPath.Path.RemoveRange(0, shortestPathPoint);
            }
        }
        else
        {
            // We don't have a path so we will slow to a stop
            if (MovementBrain.MovementController.useRootMotion)
            {
                MovementBrain.AgentController.MovementType = BaseAgentController.MovementStyle.Idle;
            }
            else
            {
                rigidbody.velocity -= rigidbody.velocity * Time.deltaTime * MovementBrain.MovementController.Acceleration;
            }
        }
    }

    private Octree.PathRequest Path
    {
        get
        {
            if ((newPath == null || newPath.isCalculating) && oldPath != null)
            {
                return oldPath;
            }
            return newPath;
        }
    }

    /// <summary>
    /// Test to see if there is a path to the current target.
    /// Note that this will return false if the path is still building,
    /// therefore you should also check Octree.IsBuilding.
    /// </summary>
    public bool HasReachableTarget
    {
        get
        {
            return Path != null && Path.Path.Count > 0;

        }
    }

    public Vector3 CurrentTargetPosition
    {
        get
        {
            if (Path != null && Path.Path.Count > 0)
            {
                return Path.Path[0];
            }
            else
            {
                return MovementBrain.Target.position;
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (rigidbody != null)
        {
            Gizmos.color = Color.blue;
            Vector3 predictedPosition = rigidbody.position + rigidbody.velocity * Time.deltaTime;
            if (collider.GetType() == typeof(SphereCollider))
            {
                Gizmos.DrawWireSphere(predictedPosition, ((SphereCollider)collider).radius);
            } else if (collider.GetType() == typeof(CapsuleCollider))
            {
                Gizmos.DrawWireSphere(predictedPosition, ((CapsuleCollider)collider).radius);
            } else
            {
                Gizmos.DrawWireCube(predictedPosition, collider.bounds.size);
            }
        }

        if (Path != null)
        {
            var path = Path;
            for (int i = 0; i < path.Path.Count - 1; i++)
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(path.Path[i], MovementBrain.MovementController.minReachDistance);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(path.Path[i], Vector3.ClampMagnitude(rigidbody.position - path.Path[i], pathPointRadius));
                Gizmos.DrawWireSphere(path.Path[i], pathPointRadius);
                Gizmos.DrawLine(path.path[i], path.Path[i + 1]);

                Octree.GetNode(path.Path[i]).DrawGizmos();
            }
        }
    }
}
