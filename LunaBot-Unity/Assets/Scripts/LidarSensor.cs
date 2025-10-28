using UnityEngine;
using System.Collections;

public class LidarSensor : MonoBehaviour {
    public int rays = 360;
    public float range = 10f;
    public float hz = 10f;
    public RosBridgeClient ros;

    IEnumerator Start(){
        while(true){
            float[] ranges = new float[rays];
            for(int i=0;i<rays;i++){
                float angle = (i/(float)rays)*360f;
                Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;
                RaycastHit hit;
                if(Physics.Raycast(transform.position, dir, out hit, range)) ranges[i] = hit.distance;
                else ranges[i] = Mathf.Infinity;
            }
            var scan = new {
                header = new {stamp = new {secs = (int)Time.time, nsecs = (int)((Time.time%1f)*1e9f)}, frame_id = "lidar"},
                angle_min = 0.0, angle_max = 2*Mathf.PI,
                angle_increment = (2*Mathf.PI)/rays,
                time_increment = 0.0,
                scan_time = 1.0f/hz,
                range_min = 0.05f, range_max = range,
                ranges = ranges
            };
            ros.Publish("/scan", scan);
            yield return new WaitForSeconds(1f/hz);
        }
    }
}