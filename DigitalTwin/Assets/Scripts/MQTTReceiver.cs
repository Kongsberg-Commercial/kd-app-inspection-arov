using UnityEngine;
using System;
using System.Collections.Concurrent;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

public class SmoothMqttReceiver : MonoBehaviour
{
    [Header("MQTT Settings")]
    public string brokerAddress = "localhost";
    public int brokerPort = 1883;
    public string pipeTopic = "/pipe";
    public string odomTopic = "/odom";
    public string pipeFollowSpeedTopic = "/pipefollowspeed";

    [Header("Smoothing Settings")]
    public float distanceLerpSpeed = 5f;
    public float angleLerpSpeed = 5f;
    public float smoothSpeed = 2f;

    [Header("Pipe Movement Settings")]
    public float distanceScale = 0.0025f;
    public float angleScale = -0.15f;

    [Header("Odometry Settings")]
    public float odomYOffset = 5.77f;

    // Internal data from MQTT
    private float rawDistance;
    private float rawAngle;
    private float odomAltitude;
    private float roll;
    private float pitch;
    private float followSpeed;  // Speed from /pipefollowspeed
    private float lastSpeedUpdateTime;  // Track last speed message time

    // Internal fields
    private MqttClient _mqttClient;
    private Vector3 _targetPosition;
    private Quaternion _targetRotation;
    private float _smoothedDistance;
    private float _smoothedAngle;

    private readonly ConcurrentQueue<(string topic, string json)> _messageQueue = new();

    // Payload data structures
    [Serializable]
    private class PipePayload
    {
        public float distance;
        public float angle;
    }

    [Serializable]
    private class OdomPayload
    {
        public Position position;
        public Velocity velocity;
        public Orientation orientation;
    }

    [Serializable]
    private class Position
    {
        public float x;
        public float y;
        public float z;
        public float zu;
    }

    [Serializable]
    private class Velocity
    {
        public float x;
        public float y;
        public float z;
        public float speed;
    }

    [Serializable]
    private class Orientation
    {
        public float roll;
        public float pitch;
        public float yaw;
    }

    [Serializable]
    private class PipeFollowSpeedPayload
    {
        public float speed;
    }

    private void Start()
    {
        // Initialize position and smoothing values
        _targetPosition = transform.position;
        _targetRotation = transform.rotation;
        _smoothedDistance = rawDistance;
        _smoothedAngle = rawAngle;

        // Initialize MQTT client and subscriptions
        _mqttClient = new MqttClient(brokerAddress, brokerPort, false, null, null, MqttSslProtocols.None);
        _mqttClient.MqttMsgPublishReceived += HandleMqttMessage;
        _mqttClient.Connect(Guid.NewGuid().ToString());
        _mqttClient.Subscribe(
            new[] { pipeTopic, odomTopic, pipeFollowSpeedTopic },
            new[] { MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE, MqttMsgBase.QOS_LEVEL_AT_LEAST_ONCE }
        );
    }

    private void HandleMqttMessage(object sender, MqttMsgPublishEventArgs e)
    {
        string message = System.Text.Encoding.UTF8.GetString(e.Message).Trim();
        Debug.Log($"[MQTT] Received on topic '{e.Topic}': {message}");
        _messageQueue.Enqueue((e.Topic, message));
    }

    private void ProcessIncomingMessages()
    {
        while (_messageQueue.TryDequeue(out var entry))
        {
            var (topic, json) = entry;

            if (topic == pipeTopic)
            {
                try
                {
                    var payload = JsonUtility.FromJson<PipePayload>(json);
                    rawDistance = payload.distance;
                    rawAngle = payload.angle;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[MQTT] Failed to parse pipe data: {ex.Message}");
                }
            }
            else if (topic == odomTopic)
            {
                try
                {
                    var payload = JsonUtility.FromJson<OdomPayload>(json);
                    odomAltitude = payload.position.z;
                    roll = payload.orientation.roll;
                    pitch = payload.orientation.pitch;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[MQTT] Failed to parse odometry data: {ex.Message}");
                }
            }
            else if (topic == pipeFollowSpeedTopic)
            {
                try
                {
                    var payload = JsonUtility.FromJson<PipeFollowSpeedPayload>(json);
                    followSpeed = payload.speed;
                    lastSpeedUpdateTime = Time.time;  // Update last speed message time
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[MQTT] Failed to parse pipe follow speed data: {ex.Message}");
                }
            }
        }
    }

    private void Update()
    {
        // Process incoming MQTT messages
        ProcessIncomingMessages();

        // Zero speed if no update within 0.5s
        if (Time.time - lastSpeedUpdateTime > 0.5f)
        {
            followSpeed = 0f;
        }

        // Smooth pipe input values
        _smoothedDistance = Mathf.Lerp(_smoothedDistance, rawDistance, Time.deltaTime * distanceLerpSpeed);
        _smoothedAngle = Mathf.Lerp(_smoothedAngle, rawAngle, Time.deltaTime * angleLerpSpeed);

        // X-axis motion from /pipefollowspeed (control signal)
        _targetPosition.x += followSpeed * 0.00035f * Time.deltaTime;  // Scale as needed

        // Update z-axis based on distance to the pipe
        _targetPosition.z = _smoothedDistance * distanceScale;

        // Apply odometry altitude (Y-axis)
        _targetPosition.y = odomAltitude + odomYOffset;

        // Normalize roll/pitch for Unity axes
        float normalizedRoll  = (Mathf.Rad2Deg * roll  - 55f) * -0.025f;
        float normalizedPitch = (Mathf.Rad2Deg * pitch - 22f) *  0.025f;
        float yaw = _smoothedAngle * angleScale;

        // Combine into target rotation
        _targetRotation = Quaternion.Euler(
            normalizedRoll,   // x-axis (pitch)
            yaw,              // y-axis (yaw)
            normalizedPitch   // z-axis (roll)
        );

        // Smoothly interpolate position and rotation
        transform.position = Vector3.Lerp(transform.position, _targetPosition, Time.deltaTime * smoothSpeed);
        transform.rotation = Quaternion.Lerp(transform.rotation, _targetRotation, Time.deltaTime * smoothSpeed);
    }

    private void OnDestroy()
    {
        if (_mqttClient?.IsConnected == true)
        {
            _mqttClient.Disconnect();
        }
    }
}
