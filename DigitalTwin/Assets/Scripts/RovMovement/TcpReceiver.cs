using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class TcpReceiver : MonoBehaviour
{
    [Header("Server Settings")]
    [Tooltip("Port number on which this server listens.")]
    public int serverPort = 8888;

    [Header("Data Received")]
    [Tooltip("Received distance value (used to move the object).")]
    public float distance;

    [Tooltip("Received angle in degrees (used for Y-axis rotation).")]
    public float angle;

    [Header("Update Settings")]
    [Tooltip("Time delay between updating the transform (in seconds).")]
    public float updateDelay = 0.1f;

    private TcpListener server;
    private TcpClient client;
    private NetworkStream stream;
    private Thread listenThread;
    private bool running = false;

    private readonly object dataLock = new object();
    private float updateTimer = 0f;

    // Smooth movement fields
    private Vector3 targetPosition;
    private float smoothSpeed = 2f;

    void Start()
    {
        targetPosition = transform.position;
        StartServer();
    }

    void StartServer()
    {
        try
        {
            server = new TcpListener(IPAddress.Parse("192.168.2.1"), serverPort);
            server.Start();
            running = true;
            Debug.Log("TCP Server started on port " + serverPort);

            listenThread = new Thread(new ThreadStart(ListenForClients));
            listenThread.IsBackground = true;
            listenThread.Start();
        }
            
        catch (Exception e)
        {
            Debug.LogError("Error starting TCP server: " + e);
        }
    }

    void ListenForClients()
    {
        try
        {
            while (running)
            {
                if (client == null)
                {
                    Debug.Log("Waiting for a client to connect...");
                    client = server.AcceptTcpClient();
                    Debug.Log("Client connected: " + client.Client.RemoteEndPoint);
                    stream = client.GetStream();

                    Thread dataThread = new Thread(new ThreadStart(ReadClientData));
                    dataThread.IsBackground = true;
                    dataThread.Start();
                }

                Thread.Sleep(100);
            }
        }
        catch (SocketException se)
        {
            Debug.LogError("Socket exception: " + se);
        }
        catch (Exception e)
        {
            Debug.LogError("Exception in ListenForClients: " + e);
        }
    }

    void ReadClientData()
    {
        byte[] buffer = new byte[1024];
        try
        {
            while (running && client != null && client.Connected)
            {
                int bytesRead = stream.Read(buffer, 0, buffer.Length);
                if (bytesRead > 0)
                {
                    string dataString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                    Debug.Log("Raw data received: " + dataString);
                    ProcessData(dataString);
                }
                else
                {
                    Debug.Log("Client disconnected.");
                    client.Close();
                    client = null;
                    break;
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError("ReadClientData error: " + e);
            if (client != null)
            {
                client.Close();
                client = null;
            }
        }
    }

    void ProcessData(string dataString)
    {
        dataString = dataString.Trim();
        string[] parts = dataString.Split(';');
        if (parts.Length >= 2)
        {
            if (float.TryParse(parts[0], out float parsedDistance) &&
                float.TryParse(parts[1], out float parsedAngle))
            {
                Debug.Log("Parsed distance: " + parsedDistance + " | Parsed angle: " + parsedAngle);
                lock (dataLock)
                {
                    targetPosition.z = parsedDistance * 0.0025f;
                    angle = parsedAngle * -0.15f ;
                }
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
        updateTimer += Time.deltaTime;
        if (updateTimer >= updateDelay)
        {
            updateTimer = 0f;

            lock (dataLock)
            {
                targetPosition.x = transform.position.x + 0.03f;
            }
        }

        transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * smoothSpeed);
        transform.rotation = Quaternion.Lerp(transform.rotation, Quaternion.Euler(0, angle, 0), Time.deltaTime * smoothSpeed);
    }

    private void OnApplicationQuit()
    {
        running = false;

        if (server != null)
        {
            server.Stop(); // unblocks AcceptTcpClient
            server = null;
        }

        if (listenThread != null && listenThread.IsAlive)
        {
            listenThread.Join();
        }

        if (client != null)
        {
            client.Close();
            client = null;
        }
    }

    public void SetInitialStateFromTransform(Transform t)
    {
        lock (dataLock)
        {
            targetPosition = t.position;
        }
    }

}
