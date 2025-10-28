using UnityEngine;
using System.Collections;

[RequireComponent(typeof(Rigidbody))]
public class RoverController : MonoBehaviour {
    public WheelCollider[] wheelColliders;
    public Transform[] wheelMeshes;
    public float maxMotorTorque = 150f;
    public float maxSteerAngle = 25f;
    public RosBridgeClient ros;
    public float odomPublishHz = 10f;
    Rigidbody rb;

    void Start(){ rb = GetComponent<Rigidbody>(); StartCoroutine(PublishOdom()); }

    IEnumerator PublishOdom(){
        while(true){
            var odom = new {
                header = new {stamp = new {secs = (int)Time.time, nsecs = (int)((Time.time%1f)*1e9f)}, frame_id = "odom"},
                pose = new { pose = new { position = new { x = transform.position.x, y = transform.position.y, z = transform.position.z}, orientation = new { x = transform.rotation.x, y = transform.rotation.y, z = transform.rotation.z, w = transform.rotation.w } }},
                twist = new { twist = new { linear = new { x = rb.velocity.x, y = rb.velocity.y, z = rb.velocity.z}, angular = new { x = rb.angularVelocity.x, y = rb.angularVelocity.y, z = rb.angularVelocity.z } }}
            };
            ros.Publish("/odom", odom);
            yield return new WaitForSeconds(1f/odomPublishHz);
        }
    }
}