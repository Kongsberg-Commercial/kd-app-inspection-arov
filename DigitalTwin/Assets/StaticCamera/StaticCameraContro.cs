using UnityEngine;
using UnityEngine.InputSystem;

public class StaticCameraControl : MonoBehaviour
{
    public float rotationSpeed = 60f;  // Degrees per second
    private float currentYaw = 0f;

    void Update()
    {
        // If ROV control is active, disable static camera input.
        if (ROVMovement.controlEnabled)
            return;

        float horizontalInput = 0f;

        // Read left/right arrow key input (using the new Input System)
        if (Keyboard.current.leftArrowKey.isPressed)
            horizontalInput = -1f;
        if (Keyboard.current.rightArrowKey.isPressed)
            horizontalInput = 1f;

        // Update current yaw based on input and rotation speed
        currentYaw += horizontalInput * rotationSpeed * Time.deltaTime;

        // Apply the rotation (only affect the Y-axis)
        transform.rotation = Quaternion.Euler(9f, currentYaw, 0f);
    }
}
