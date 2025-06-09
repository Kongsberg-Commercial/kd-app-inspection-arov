using UnityEngine;
using System;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class RovDataReceiver : MonoBehaviour
{
    [Header("Connection Settings")]
    [Tooltip("IP address of the server (where the Python script is running)")]
    public string serverIP = "192.168.2.1";
    [Tooltip("Port number (must match the Python script).")]
    public int serverPort = 8888;

    [Header("ROV Reference")]
    [Tooltip("Assign the Transform of your ROV here.")]
    public Transform rovTransform;

    // Parsed data: horizontal offset (y_e) and angle in degrees (phi_e_deg)
    public float yOffset;
    public float phiDeg;

    private TcpClient client;
    private NetworkStream stream;
    private Thread receiveThread;
    private bool isConnected = false;

    void Start()
    {
        ConnectToServer();
    }

    /// <summary>
    /// Connect to the server using TCP.
    /// </summary>
    private void ConnectToServer()
    {
        try
        {
            client = new TcpClient(serverIP, serverPort);
            stream = client.GetStream();
            isConnected = true;
            // Start receiving data in a background thread.
            receiveThread = new Thread(new ThreadStart(ReceiveData));
            receiveThread.IsBackground = true;
            receiveThread.Start();
            Debug.Log("Connected to server at " + serverIP + ":" + serverPort);
        }
        catch (Exception e)
        {
            Debug.LogError("Error connecting to server: " + e);
        }
    }

    /// <summary>
    /// Continuously receive data from the server on a separate thread.
    /// </summary>
    private void ReceiveData()
    {
        byte[] buffer = new byte[1024];
        while (isConnected)
        {
            try
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string dataString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    // Log the raw incoming data.
                    Debug.Log("Raw data received: " + dataString);
                    ProcessData(dataString);
                }
            }
            catch (Exception e)
            {
                Debug.LogError("ReceiveData error: " + e);
                isConnected = false;
            }
        }
    }

    /// <summary>
    /// Parse the incoming data string into float values.
    /// Expected format: "y_e;phi_e_deg\n"
    /// </summary>
    /// <param name="dataString"></param>
    private void ProcessData(string dataString)
    {
        // Clean up the data string (remove newlines, etc.)
        dataString = dataString.Trim();
        string[] parts = dataString.Split(';');
        if (parts.Length >= 2)
        {
            if (float.TryParse(parts[0], out float parsedY) &&
                float.TryParse(parts[1], out float parsedPhi))
            {
                // Log the parsed values.
                Debug.Log("Parsed yOffset: " + parsedY + " | Parsed phiDeg: " + parsedPhi);
                yOffset = parsedY;
                phiDeg = parsedPhi;
            }
            else
            {
                Debug.LogWarning("Failed to parse data: " + dataString);
            }
        }
        else
        {
            Debug.LogWarning("Received data does not have enough parts: " + dataString);
        }
    }

    void Update()
    {
        if (rovTransform != null)
        {
            // Update the ROV's transform based on the received data.
            Vector3 pos = rovTransform.position;
            pos.x = yOffset; // (Assuming your coordinate system: adjust as needed)
            rovTransform.position = pos;

            // Update the Y-axis rotation using phiDeg.
            rovTransform.rotation = Quaternion.Euler(0, phiDeg, 0);
        }
    }

    private void OnApplicationQuit()
    {
        isConnected = false;
        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Abort();
        }
        if (stream != null)
            stream.Close();
        if (client != null)
            client.Close();
    }
}
