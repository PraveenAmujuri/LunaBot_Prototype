using UnityEngine;
using System.Collections;
using System.Text.RegularExpressions;
using System.Collections.Concurrent;

[RequireComponent(typeof(Rigidbody))]
public class ROSControlledRover : MonoBehaviour
{
    public RosBridgeClient ros;

    [Header("Movement Settings")]
    public float moveForce = 1200f;      // /Tuing: adjust for acceleration strength
    public float turnTorque = 600f;      // /Tuing: adjust turning responsiveness

    [Header("Physics Settings")]
    public float groundCheckDistance = 2f;
    public LayerMask groundLayer;

    private Rigidbody rb;
    private bool odomAdvertised = false;
    private bool cmdVelSubscribed = false;
    private bool tfAdvertised = false;

    private struct RosCommand
    {
        public float linear;
        public float angular;
    }

    private ConcurrentQueue<RosCommand> commandQueue = new ConcurrentQueue<RosCommand>();

    private float currentLinearCommand = 0f;
    private float currentAngularCommand = 0f;
    private int lastCommandFrame = 0;
    private Vector3 lastPosition;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.centerOfMass = new Vector3(0, -0.5f, 0);   // /Tuing: lower center of mass for stability
        rb.mass = 60f;
        rb.drag = 2f;
        rb.angularDrag = 8f;

        lastPosition = transform.position;

        if (ros != null)
        {
            StartCoroutine(PublishOdom());
            StartCoroutine(SubscribeToROSCommands());
        }
        else
        {
            Debug.LogError("ROS Bridge not assigned");
        }
    }

    IEnumerator SubscribeToROSCommands()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);
        yield return new WaitForSeconds(2f);

        ros.OnCmdVelReceived = HandleRosCmdVel;
        ros.Subscribe("/cmd_vel", "geometry_msgs/Twist");
        cmdVelSubscribed = true;
    }

    void HandleRosCmdVel(object message)
    {
        try
        {
            string msgString = message.ToString();

            float rosLinear = 0f, rosAngular = 0f;

            Match linearMatch = Regex.Match(msgString,
                @"""linear"":\s*\{\s*""x"":\s*([-+]?\d*\.?\d+)");
            if (linearMatch.Success)
                float.TryParse(linearMatch.Groups[1].Value, out rosLinear);

            Match angularMatch = Regex.Match(msgString,
                @"""angular"":\s*\{[^}]*""z"":\s*([-+]?\d*\.?\d+)");
            if (angularMatch.Success)
                float.TryParse(angularMatch.Groups[1].Value, out rosAngular);

            float unityLinear = Mathf.Clamp(rosLinear, -0.6f, 0.6f);     // /Tuing: max wheel speed
            float unityAngular = Mathf.Clamp(-rosAngular, -0.4f, 0.4f); // /Tuing: turn limit

            if (Mathf.Abs(unityAngular) < 0.08f)
                unityAngular = 0f;  // /Tuing: dead zone reduces jitter

            var command = new RosCommand
            {
                linear = unityLinear,
                angular = unityAngular
            };

            commandQueue.Enqueue(command);
        }
        catch { }
    }

    void Update()
    {
        ProcessCommandQueue();

        if (Input.GetKey(KeyCode.M)) SetDirectCommand(0.5f, 0f);
        else if (Input.GetKey(KeyCode.S)) SetDirectCommand(-0.5f, 0f);
        else if (Input.GetKey(KeyCode.A)) SetDirectCommand(0f, 0.4f);
        else if (Input.GetKey(KeyCode.D)) SetDirectCommand(0f, -0.4f);
        else if (Input.GetKeyDown(KeyCode.X)) SetDirectCommand(0f, 0f);
    }

    void ProcessCommandQueue()
    {
        while (commandQueue.TryDequeue(out RosCommand command))
        {
            currentLinearCommand = command.linear;
            currentAngularCommand = command.angular;
            lastCommandFrame = Time.frameCount;
        }
    }

    void FixedUpdate()
    {
        ApplyBasicSuspension();
        ExecuteCommands();
        ApplyStabilization();
    }

    void ExecuteCommands()
    {
        if (Time.frameCount - lastCommandFrame > 120)
        {
            currentLinearCommand = 0f;          // /Tuing: fail-safe timeout for lost connection
            currentAngularCommand = 0f;
        }

        if (Mathf.Abs(currentLinearCommand) > 0.01f)
            rb.AddForce(transform.forward * currentLinearCommand * moveForce);

        if (Mathf.Abs(currentAngularCommand) > 0.01f)
            rb.AddTorque(Vector3.up * currentAngularCommand * turnTorque);

        currentLinearCommand *= 0.96f;   // /Tuing: decay for smoother stopping
        currentAngularCommand *= 0.92f;
    }

    public void SetDirectCommand(float linear, float angular)
    {
        currentLinearCommand = Mathf.Clamp(linear, -1f, 1f);
        currentAngularCommand = Mathf.Clamp(angular, -1f, 1f);
        lastCommandFrame = Time.frameCount;
    }

    void ApplyBasicSuspension()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, -transform.up, out hit, groundCheckDistance, groundLayer))
        {
            float compression = groundCheckDistance - hit.distance;
            float upwardVelocity = Vector3.Dot(rb.velocity, Vector3.up);
            float force = compression * 600f - upwardVelocity * 50f;
            rb.AddForce(Vector3.up * force);   // /Tuing: adjust force multiplier for damping softness
        }
    }

    void ApplyStabilization()
    {
        Vector3 sidewaysVelocity = Vector3.Project(rb.velocity, transform.right);
        if (sidewaysVelocity.magnitude > 0.1f)
            rb.AddForce(-sidewaysVelocity * 30f);  // /Tuing: tweak for drift correction

        Vector3 angularVel = rb.angularVelocity;
        angularVel.x *= 0.7f;
        angularVel.z *= 0.7f;
        angularVel.y = Mathf.Clamp(angularVel.y, -1.5f, 1.5f); // /Tuing: clamp yaw rate
        rb.angularVelocity = angularVel;
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

            if (!tfAdvertised)
            {
                ros.AdvertiseTopic("/tf", "tf2_msgs/TFMessage");
                tfAdvertised = true;
            }

            Vector3 p = transform.position;

            float rosX = p.z;
            float rosY = -p.x;
            float rosZ = p.y;

            float rosYaw = -transform.eulerAngles.y * Mathf.Deg2Rad;

            var odomMsg = new
            {
                header = new
                {
                    stamp = new
                    {
                        sec = (int)Time.time,
                        nanosec = (int)((Time.time % 1) * 1e9)
                    },
                    frame_id = "odom"
                },
                child_frame_id = "base_link",
                pose = new
                {
                    pose = new
                    {
                        position = new { x = rosX, y = rosY, z = rosZ },
                        orientation = new
                        {
                            z = Mathf.Sin(rosYaw / 2),
                            w = Mathf.Cos(rosYaw / 2)
                        }
                    }
                },
                twist = new
                {
                    twist = new
                    {
                        linear = new
                        {
                            x = rb.velocity.z,
                            y = -rb.velocity.x,
                            z = rb.velocity.y
                        },
                        angular = new { z = -rb.angularVelocity.y }
                    }
                }
            };

            ros.Publish("/odom", odomMsg);

            PublishTF(rosX, rosY, rosZ,
                0f, 0f,
                Mathf.Sin(rosYaw / 2),
                Mathf.Cos(rosYaw / 2));

            lastPosition = p;

            yield return new WaitForSeconds(0.1f);   // /Tuing: odom rate (10Hz)
        }
    }

    void PublishTF(float rosX, float rosY, float rosZ, float qx, float qy, float qz, float qw)
    {
        var tfMsg = new
        {
            transforms = new[]
            {
                new {
                    header = new {
                        stamp = new {
                            sec = (int)Time.time,
                            nanosec = (int)((Time.time % 1) * 1e9)
                        },
                        frame_id = "odom"
                    },
                    child_frame_id = "base_link",
                    transform = new
                    {
                        translation = new { x = rosX, y = rosY, z = rosZ },
                        rotation = new { x = qx, y = qy, z = qz, w = qw }
                    }
                }
            }
        };

        ros.Publish("/tf", tfMsg);
    }
}
