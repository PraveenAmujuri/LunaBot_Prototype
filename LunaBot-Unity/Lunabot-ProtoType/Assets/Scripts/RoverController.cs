using UnityEngine;
using System.Collections;

[RequireComponent(typeof(Rigidbody))]
public class RoverController : MonoBehaviour
{
    public float speed = 2f;
    public float rotationSpeed = 50f;

    public RosBridgeClient ros;
    public float odomPublishHz = 10f;

    private Rigidbody rb;
    private bool odomAdvertised = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;

        if (ros != null)
        {
            StartCoroutine(PublishOdom());
        }
        else
        {
            Debug.LogWarning("RosBridgeClient not assigned!");
        }
    }

    void FixedUpdate()
    {
        // Manual keyboard control for testing
        float moveInput = Input.GetAxis("Vertical");
        float turnInput = Input.GetAxis("Horizontal");

        Vector3 move = transform.forward * moveInput * speed * Time.fixedDeltaTime;
        rb.MovePosition(rb.position + move);

        float turn = turnInput * rotationSpeed * Time.fixedDeltaTime;
        rb.MoveRotation(rb.rotation * Quaternion.Euler(0f, turn, 0f));
    }

    IEnumerator PublishOdom()
    {
        while (true)
        {
            // Wait until the ROS bridge is connected
            yield return new WaitUntil(() => ros != null && ros.IsConnected);

            // Advertise /odom once
            if (!odomAdvertised)
            {
                ros.AdvertiseTopic("/odom", "nav_msgs/Odometry");
                odomAdvertised = true;
                Debug.Log("Advertised /odom");
            }

            // Prepare odometry message
            var odomMsg = new
            {
                header = new { stamp = new { sec = (int)Time.time, nanosec = (int)((Time.time % 1) * 1e9) }, frame_id = "odom" },
                child_frame_id = "base_link",
                pose = new
                {
                    pose = new
                    {
                        position = new { x = transform.position.x, y = transform.position.y, z = transform.position.z },
                        orientation = new { x = transform.rotation.x, y = transform.rotation.y, z = transform.rotation.z, w = transform.rotation.w }
                    }
                },
                twist = new
                {
                    twist = new
                    {
                        linear = new { x = rb.velocity.z, y = 0f, z = -rb.velocity.x }, // Example of converting Unity velocity to ROS
                        angular = new { x = 0f, y = -rb.angularVelocity.y, z = 0f }
                    }
                }
            };

            ros.Publish("/odom", odomMsg);

            yield return new WaitForSeconds(1f / odomPublishHz);
        }
    }
}