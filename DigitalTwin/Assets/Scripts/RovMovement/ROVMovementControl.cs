using UnityEngine;
using UnityEngine.InputSystem;

public class ROVControlManager : MonoBehaviour
{
    [Header("Input Action for Toggling Control Mode")]
    [Tooltip("Reference to the Input System action (bound to 'L' key).")]
    public InputActionReference localRemoteAction;

    [Header("Control Scripts")]
    [Tooltip("Handles incoming MQTT messages for remote control.")]
    public SmoothMqttReceiver mqttReceiver;
    [Tooltip("Handles direct local movement input.")]
    public ROVMovement rovMovement;

    [Header("Local/Remote Control")]
    [Tooltip("When true, control comes from MQTT messages; when false, local input controls the ROV.")]
    public bool useRemoteControl = false;

    private bool lastControlMode;

    private void OnEnable()
    {
        if (localRemoteAction?.action != null)
        {
            localRemoteAction.action.performed += OnTogglePerformed;
            localRemoteAction.action.Enable();
        }
    }

    private void OnDisable()
    {
        if (localRemoteAction?.action != null)
        {
            localRemoteAction.action.performed -= OnTogglePerformed;
            localRemoteAction.action.Disable();
        }
    }

    private void Start()
    {
        lastControlMode = !useRemoteControl; // Force mode to apply on start
        ApplyControlMode();
    }

    private void OnTogglePerformed(InputAction.CallbackContext ctx)
    {
        useRemoteControl = !useRemoteControl;
        Debug.Log("Control mode toggled: " + (useRemoteControl ? "Remote (MQTT)" : "Local"));

        if (useRemoteControl != lastControlMode)
        {
            ApplyControlMode();
            lastControlMode = useRemoteControl;
        }
    }

    private void ApplyControlMode()
    {
        if (useRemoteControl)
        {
            if (mqttReceiver != null)
            {
                mqttReceiver.enabled = true;
                //mqttReceiver.SetInitialStateFromTransform(transform);
            }
            if (rovMovement != null)
                rovMovement.enabled = false;
        }
        else
        {
            if (mqttReceiver != null)
                mqttReceiver.enabled = false;
            if (rovMovement != null)
                rovMovement.enabled = true;
        }
    }
}
