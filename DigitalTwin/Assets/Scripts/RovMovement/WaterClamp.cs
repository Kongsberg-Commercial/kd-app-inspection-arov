using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class WaterClamp : MonoBehaviour
{
    public float waterLevelY = 10f; // The Y position of your water plane
    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        Vector3 pos = rb.position;

        // If the box is above water, clamp it to waterLevelY
        if (pos.y > waterLevelY)
        {
            pos.y = waterLevelY;
            rb.position = pos;

            // Optionally zero out vertical velocity
            Vector3 vel = rb.linearVelocity;
            if (vel.y > 0) vel.y = 0f;
            rb.linearVelocity = vel;
        }
    }
}
