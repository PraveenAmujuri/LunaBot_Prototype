using UnityEngine;
using System.Collections;
using System.Text.RegularExpressions;
using System.Collections.Concurrent;

[RequireComponent(typeof(Rigidbody))]
public class ROSControlledRover : MonoBehaviour
{
    public RosBridgeClient ros;

    [Header("Movement Settings")]
    public float moveForce = 1200f;
    public float turnTorque = 600f;

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
        rb.centerOfMass = new Vector3(0, -0.5f, 0);
        rb.mass = 60f;
        rb.drag = 2f;
        rb.angularDrag = 8f;
        lastPosition = transform.position;

        if (ros != null)
        {
            StartCoroutine(PublishOdom());
            StartCoroutine(SubscribeToROSCommands());
            Debug.Log("🤖 Simple ROS Rover - Ready!");
        }
        else
        {
            Debug.LogError("❌ ROS Bridge not assigned!");
        }
    }

    IEnumerator SubscribeToROSCommands()
    {
        yield return new WaitUntil(() => ros != null && ros.IsConnected);
        yield return new WaitForSeconds(2f);

        ros.OnCmdVelReceived = HandleRosCmdVel;
        ros.Subscribe("/cmd_vel", "geometry_msgs/Twist");
        cmdVelSubscribed = true;

        Debug.Log("🔗 Unity subscribed to ROS /cmd_vel (Simple)");
    }

    void HandleRosCmdVel(object message)
    {
        try
        {
            string msgString = message.ToString();

            float rosLinear = 0f, rosAngular = 0f;

            Match linearMatch = Regex.Match(msgString, @"""linear"":\s*\{\s*""x"":\s*([-+]?\d*\.?\d+)");
            if (linearMatch.Success)
                float.TryParse(linearMatch.Groups[1].Value, out rosLinear);

            Match angularMatch = Regex.Match(msgString, @"""angular"":\s*\{[^}]*""z"":\s*([-+]?\d*\.?\d+)");
            if (angularMatch.Success)
                float.TryParse(angularMatch.Groups[1].Value, out rosAngular);

            float unityLinear = Mathf.Clamp(rosLinear, -0.6f, 0.6f);
            float unityAngular = Mathf.Clamp(-rosAngular, -0.4f, 0.4f);

            if (Mathf.Abs(unityAngular) < 0.08f)
                unityAngular = 0f;

            var command = new RosCommand
            {
                linear = unityLinear,
                angular = unityAngular
            };

            commandQueue.Enqueue(command);

            Debug.Log("🧠 ROS→Unity: L=" + unityLinear.ToString("F2") + ", A=" + unityAngular.ToString("F2"));
        }
        catch (System.Exception e)
        {
            Debug.LogError("❌ ROS cmd_vel parsing error: " + e.Message);
        }
    }

    void Update()
    {
        ProcessCommandQueue();

        if (Input.GetKey(KeyCode.M))
            SetDirectCommand(0.5f, 0f);
        else if (Input.GetKey(KeyCode.S))
            SetDirectCommand(-0.5f, 0f);
        else if (Input.GetKey(KeyCode.A))
            SetDirectCommand(0f, 0.4f);
        else if (Input.GetKey(KeyCode.D))
            SetDirectCommand(0f, -0.4f);
        else if (Input.GetKeyDown(KeyCode.X))
        {
            SetDirectCommand(0f, 0f);
            Debug.Log("🛑 Emergency stop");
        }

        if (Input.GetKeyDown(KeyCode.Space))
            CheckROSStatus();
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
            currentLinearCommand = 0f;
            currentAngularCommand = 0f;
        }

        if (Mathf.Abs(currentLinearCommand) > 0.01f)
        {
            Vector3 forwardForce = transform.forward * currentLinearCommand * moveForce;
            rb.AddForce(forwardForce);
        }

        if (Mathf.Abs(currentAngularCommand) > 0.01f)
        {
            Vector3 torque = Vector3.up * currentAngularCommand * turnTorque;
            rb.AddTorque(torque);
        }

        currentLinearCommand *= 0.96f;
        currentAngularCommand *= 0.92f;
    }

    public void SetDirectCommand(float linear, float angular)
    {
        currentLinearCommand = Mathf.Clamp(linear, -1f, 1f);
        currentAngularCommand = Mathf.Clamp(angular, -1f, 1f);
        lastCommandFrame = Time.frameCount;
    }

    void CheckROSStatus()
    {
        if (ros != null && ros.IsConnected)
        {
            Debug.Log("✅ ROS Status: CONNECTED");
            Debug.Log("🎮 Current Commands: Linear=" + currentLinearCommand.ToString("F2") + ", Angular=" + currentAngularCommand.ToString("F2"));
            Debug.Log("🏃 Rover Speed: " + rb.velocity.magnitude.ToString("F2") + " m/s");
            Debug.Log("📍 Position: " + transform.position.ToString());
        }
        else
            Debug.Log("❌ ROS Status: NOT CONNECTED");
    }

    void ApplyBasicSuspension()
    {
        RaycastHit hit;
        if (Physics.Raycast(transform.position, -transform.up, out hit, groundCheckDistance, groundLayer))
        {
            float compression = groundCheckDistance - hit.distance;
            float upwardVelocity = Vector3.Dot(rb.velocity, Vector3.up);
            float force = compression * 600f - upwardVelocity * 50f;
            rb.AddForce(Vector3.up * force);
        }
    }

    void ApplyStabilization()
    {
        Vector3 sidewaysVelocity = Vector3.Project(rb.velocity, transform.right);
        if (sidewaysVelocity.magnitude > 0.1f)
            rb.AddForce(-sidewaysVelocity * 30f);

        Vector3 angularVel = rb.angularVelocity;
        angularVel.x *= 0.7f;
        angularVel.z *= 0.7f;
        angularVel.y = Mathf.Clamp(angularVel.y, -1.5f, 1.5f);
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
                Debug.Log("📡 Simple odometry ready");
            }

            if (!tfAdvertised)
            {
                ros.AdvertiseTopic("/tf", "tf2_msgs/TFMessage");
                tfAdvertised = true;
                Debug.Log("📡 TF topic advertised");
            }

            // FIXED: Always publish odometry and TF at 10 Hz, regardless of movement
            Vector3 currentPos = transform.position;
            float rosX = currentPos.z;
            float rosY = -currentPos.x;
            float rosZ = currentPos.y;
            float rosYaw = -transform.eulerAngles.y * Mathf.Deg2Rad;

            var odomMsg = new
            {
                header = new
                {
                    stamp = new { sec = (int)Time.time, nanosec = (int)((Time.time % 1) * 1e9) },
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
                            x = 0f, y = 0f,
                            z = Mathf.Sin(rosYaw / 2),
                            w = Mathf.Cos(rosYaw / 2)
                        }
                    }
                },
                twist = new
                {
                    twist = new
                    {
                        linear = new { x = rb.velocity.z, y = -rb.velocity.x, z = rb.velocity.y },
                        angular = new { x = 0f, y = 0f, z = -rb.angularVelocity.y }
                    }
                }
            };

            ros.Publish("/odom", odomMsg);
            PublishTF(rosX, rosY, rosZ, 0f, 0f, Mathf.Sin(rosYaw / 2), Mathf.Cos(rosYaw / 2));
            lastPosition = currentPos;

            // Publish at 10 Hz (every 0.1 seconds)
            yield return new WaitForSeconds(0.1f);
        }
    }

    void PublishTF(float rosX, float rosY, float rosZ, float qx, float qy, float qz, float qw)
    {
        var tfMsg = new
        {
            transforms = new[] {
                new {
                    header = new {
                        stamp = new { sec = (int)Time.time, nanosec = (int)((Time.time % 1) * 1e9) },
                        frame_id = "odom"
                    },
                    child_frame_id = "base_link",
                    transform = new {
                        translation = new { x = rosX, y = rosY, z = rosZ },
                        rotation = new { x = qx, y = qy, z = qz, w = qw }
                    }
                }
            }
        };
        ros.Publish("/tf", tfMsg);
    }
}
