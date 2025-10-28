using UnityEngine;
using System.Collections;

public class PatrolController : MonoBehaviour {
    public Transform[] waypoints;
    public float speed = 1.0f;
    public bool patrolling = false;

    public IEnumerator PatrolLoop(){
        int idx = 0;
        while(patrolling){
            Transform wp = waypoints[idx];
            while(Vector3.Distance(transform.position, wp.position) > 0.5f){
                Vector3 dir = (wp.position - transform.position).normalized;
                transform.position += dir * speed * Time.deltaTime;
                yield return null;
            }
            idx = (idx+1)%waypoints.Length;
            yield return new WaitForSeconds(0.5f);
        }
    }
}