using UnityEngine;
using UnityEngine.InputSystem;

public class CameraSwitcher : MonoBehaviour
{
    public Camera playerCamera;   // Assign your player camera in the Inspector
    public Camera staticCamera;   // Assign your static camera in the Inspector

    private bool usingPlayerCamera = true;

    private void Start()
    {
        if (playerCamera != null)
            playerCamera.enabled = true;
        if (staticCamera != null)
            staticCamera.enabled = false;

        // Enable ROV control when starting with the player camera.
        ROVMovement.controlEnabled = true;
    }

    private void Update()
    {
        // Toggle cameras when pressing the "C" key.
        if (Keyboard.current.cKey.wasPressedThisFrame)
        {
            usingPlayerCamera = !usingPlayerCamera;

            if (playerCamera != null)
                playerCamera.enabled = usingPlayerCamera;
            if (staticCamera != null)
                staticCamera.enabled = !usingPlayerCamera;

            // When using the player camera, enable ROV controls; disable them when using the static camera.
            ROVMovement.controlEnabled = usingPlayerCamera;
        }
    }
}
