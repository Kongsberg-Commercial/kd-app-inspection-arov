using UnityEngine;

public class FishWander : MonoBehaviour
{
    [Header("Wander Settings")]
    public float swimSpeed = 2f;                // How fast the fish swims forward.
    [Tooltip("Slerp factor for smooth rotation (0 = no rotation, 1 = immediate change).")]
    public float turnSpeed = 0.5f;              
    [Tooltip("Time in seconds between direction changes.")]
    public float changeDirInterval = 3f;        
    [Tooltip("After a collision, delay the next direction change by this amount.")]
    public float collisionDelay = 5f;           

    private float timer;
    private Quaternion targetRotation;          // The current target rotation.

    void Start()
    {
        timer = changeDirInterval;
        // Start with the current rotation as the target.
        targetRotation = transform.rotation;
    }

    void Update()
    {
        // Move forward continuously.
        transform.Translate(Vector3.forward * swimSpeed * Time.deltaTime);

        // Smoothly rotate toward the target rotation.
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, turnSpeed * Time.deltaTime);

        // Decrease timer.
        timer -= Time.deltaTime;
        if (timer <= 0f)
        {
            // Choose a new random horizontal direction.
            Vector3 randomDir = new Vector3(Random.Range(-1f, 1f), 0f, Random.Range(-1f, 1f)).normalized;
            // If for some reason randomDir is zero, fall back to the current forward direction.
            if(randomDir == Vector3.zero)
            {
                randomDir = transform.forward;
            }
            targetRotation = Quaternion.LookRotation(randomDir);

            // Reset timer to the default change interval.
            timer = changeDirInterval;
        }
    }

    // When a collision occurs, delay the next direction change.
    private void OnCollisionEnter(Collision collision)
    {
        // Set timer to collisionDelay if it's less than that.
        if (timer < collisionDelay)
        {
            timer = collisionDelay;
        }
    }
}
