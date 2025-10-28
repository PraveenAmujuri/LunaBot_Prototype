using UnityEngine;
using System.Collections;

[RequireComponent(typeof(Rigidbody))]
public class RoverAutonomousController : MonoBehaviour
{
    public RosBridgeClient ros;
    public float speed = 150f;
    public float turnSpeed = 100f;
    public float waypointThreshold = 2.0f;
    public Transform[] waypoints;
    
    public float groundCheckDistance = 2f;
    public LayerMask groundLayer;
    public float alignmentTorque = 100f;

    private int currentWaypoint = 0;
    private Rigidbody rb;
    private bool odomAdvertised = false;
    private bool cmdVelAdvertised = false;
    private bool hazardAdvertised = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.centerOfMass = new Vector3(0, -0.5f, 0);

        if (ros != null)
        {
            StartCoroutine(PublishOdom());
        }
    }

    void FixedUpdate()
    {
        ApplySuspension();
        HandleNavigation();
        CorrectSidewaysSliding();
        PublishCmdVel();
    }

    void Update()
    {
        // Test hazard from main rover controller
        if (Input.GetKeyDown(KeyCode.H))
        {
            DebugROSConnection();
            PublishHazardAlert("🚨 ROVER CONTROLLER TEST", "rover_test");
        }
        
        if (Input.GetKeyDown(KeyCode.G))
        {
            DebugROSConnection();
            PublishHazardAlert("⚡ ROVER G KEY TEST", "rover_g_test");
        }
        
        if (Input.GetKeyDown(KeyCode.R))
        {
            DebugROSConnection();
        }
    }

    void DebugROSConnection()
    {
        if (ros != null)
        {
            Debug.Log($"🔌 ROS Connected: {ros.IsConnected}");
            Debug.Log($"🔧 ROS Object Found: YES");
        }
        else
        {
            Debug.Log("❌ ROS is NULL - Check inspector assignment!");
        }
    }

    // FIXED HAZARD METHOD - Using correct std_msgs/String format
    public void PublishHazardAlert(string message, string alertType = "rover_alert")
    {
        if (ros != null && ros.IsConnected)
        {
            if (!hazardAdvertised)
            {
                ros.AdvertiseTopic("/hazard", "std_msgs/String");
                hazardAdvertised = true;
                Debug.Log("📡 Hazard topic advertised by rover controller");
            }

            // FIXED: Use correct std_msgs/String format with 'data' field
            var hazardMsg = new 
            {
                data = $"{message} [Type: {alertType}]"
            };

            ros.Publish("/hazard", hazardMsg);
            Debug.Log($"✅ ROVER HAZARD SENT: {message}");
        }
        else
        {
            Debug.LogWarning("❌ ROS not connected - cannot send hazard from rover");
        }
    }

    void ApplySuspension()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, -transform.up, out hit, groundCheckDistance, groundLayer))
        {
            Vector3 alignmentAxis = Vector3.Cross(transform.up, hit.normal);
            alignmentAxis.y = 0;
            rb.AddTorque(alignmentAxis * alignmentTorque);

            float compression = groundCheckDistance - hit.distance;
            float upwardVelocity = Vector3.Dot(rb.velocity, Vector3.up);
            float force = compression * 500f - upwardVelocity * 50f;
            rb.AddForce(Vector3.up * force);
        }
    }
    
    void HandleNavigation()
    {
        if (waypoints.Length == 0 || currentWaypoint >= waypoints.Length) 
        {
            return;
        }

        Transform target = waypoints[currentWaypoint];
        Vector3 roverPos = transform.position;
        Vector3 targetPos = target.position;
        
        Vector3 directionToTarget = (targetPos - roverPos);
        directionToTarget.y = 0;
        float distanceToTarget = directionToTarget.magnitude;

        if (distanceToTarget < waypointThreshold)
        {
            currentWaypoint = (currentWaypoint + 1) % waypoints.Length;
            return;
        }

        if (distanceToTarget > 0.1f)
        {
            directionToTarget = directionToTarget.normalized;

            float angle = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);

            if (Mathf.Abs(angle) > 10f)
            {
                float steerInput = Mathf.Sign(angle) * 0.5f;
                rb.AddTorque(Vector3.up * steerInput * turnSpeed);
            }

            float alignment = Vector3.Dot(transform.forward, directionToTarget);
            
            if (alignment > 0.5f)
            {
                rb.AddForce(transform.forward * speed * 0.5f);
            }
        }

        Debug.DrawLine(roverPos, targetPos, Color.yellow, 0.1f);
    }

    void CorrectSidewaysSliding()
    {
        Vector3 sidewaysVelocity = Vector3.Project(rb.velocity, transform.right);
        
        if (sidewaysVelocity.magnitude > 0.1f)
        {
            rb.AddForce(-sidewaysVelocity * 30f);
        }
        
        Vector3 angularVel = rb.angularVelocity;
        angularVel.x *= 0.9f;
        angularVel.z *= 0.9f;
        angularVel.y = Mathf.Clamp(angularVel.y, -2f, 2f);
        rb.angularVelocity = angularVel;
    }

    void PublishCmdVel()
    {
        if (ros != null && ros.IsConnected)
        {
            if (!cmdVelAdvertised)
            {
                ros.AdvertiseTopic("/cmd_vel", "geometry_msgs/Twist");
                cmdVelAdvertised = true;
            }
            float linearVelocity = Vector3.Dot(rb.velocity, transform.forward);
            float angularVelocity = rb.angularVelocity.y;

            var twistMsg = new
            {
                linear = new { x = linearVelocity, y = 0f, z = 0f },
                angular = new { x = 0f, y = 0f, z = angularVelocity }
            };
            ros.Publish("/cmd_vel", twistMsg);
        }
    }

    IEnumerator PublishOdom()
    {
        while (true)
        {
            yield return new WaitUntil(() => ros != null && ros.IsConnected);
            if (!odomAdvertised)
            {
                ros.AdvertiseTopic("/odom", "nav_msgs/Odometry");
                odomAdvertised = true;
            }
            var odomMsg = new
            {
                header = new { stamp = new { sec = (int)Time.time, nanosec = (int)((Time.time % 1) * 1e9) }, frame_id = "odom" },
                child_frame_id = "base_link",
                pose = new { pose = new {
                    position = new { x = transform.position.x, y = transform.position.y, z = transform.position.z },
                    orientation = new { x = transform.rotation.x, y = transform.rotation.y, z = transform.rotation.z, w = transform.rotation.w }
                }},
                twist = new { twist = new {
                    linear = new { x = rb.velocity.x, y = rb.velocity.y, z = rb.velocity.z },
                    angular = new { x = rb.angularVelocity.x, y = rb.angularVelocity.y, z = rb.angularVelocity.z }
                }}
            };
            ros.Publish("/odom", odomMsg);
            yield return new WaitForSeconds(0.1f);
        }
    }
}
