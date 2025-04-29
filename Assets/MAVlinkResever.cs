using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using static MAVLink;

public class MAVlinkResever: MonoBehaviour
{
    [Header("UDP Settings")]
    public int localPort = 14551; // Порт для приёма MAVLink
    private UdpClient udpClient;
    private Thread receiveThread;
    private bool isRunning = true;
    private MavlinkParse mavlinkParser; // Парсер MAVLink


    [Header("MAVLink Data")]
    public float roll;    // Крен (радианы)
    public float pitch;   // Тангаж (радианы)
    public float yaw;     // Рысканье (радианы)
    public float altitude; // Высота (метры)
    public Vector3 position; // GPS-позиция (latitude, longitude, altitude)

    void Start()
    {
        mavlinkParser = new MavlinkParse();
        StartUDPListener();
    }

    void OnDestroy()
    {
        StopUDPListener();
    }

    private void StartUDPListener()
    {
        udpClient = new UdpClient(new IPEndPoint(IPAddress.Any, localPort));
        receiveThread = new Thread(new ThreadStart(ReceiveData));
        receiveThread.IsBackground = true;
        receiveThread.Start();
        Debug.Log($"MAVLink UDP-приёмник запущен на порту {localPort}");
    }

    private void StopUDPListener()
    {
        isRunning = false;
        udpClient?.Close();
        receiveThread?.Abort();
        Debug.Log("MAVLink UDP-приёмник остановлен");
    }

    private void ReceiveData()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
        while (isRunning)
        {
            try
            {
                byte[] receivedBytes = udpClient.Receive(ref remoteEndPoint);
                MAVLinkMessage msg = mavlinkParser.ReadPacket(new System.IO.MemoryStream(receivedBytes));
                ProcessMavlinkMessage(msg);
            }
            catch (Exception e)
            {
                Debug.LogError($"Ошибка приёма MAVLink: {e.Message}");
            }
        }
    }

    private void ProcessMavlinkMessage(MAVLinkMessage msg)
    {
        switch (msg.msgid)
        {
            case (uint)MAVLINK_MSG_ID.ATTITUDE:
                var attitude = (mavlink_attitude_t)msg.data;
                roll = attitude.roll;
                pitch = attitude.pitch;
                yaw = attitude.yaw;
                Debug.Log($"Attitude: Roll={roll}, Pitch={pitch}, Yaw={yaw}");
                break;

            case (uint)MAVLINK_MSG_ID.GLOBAL_POSITION_INT:
                var gps = (mavlink_global_position_int_t)msg.data;
                altitude = gps.alt / 1000f; // Из мм в метры
                position = new Vector3(
                    gps.lat / 1e7f,  // Широта (deg)
                    gps.lon / 1e7f,  // Долгота (deg)
                    altitude
                );
                Debug.Log($"GPS: Lat={position.x}, Lon={position.y}, Alt={altitude}m");
                break;

            case (uint)MAVLINK_MSG_ID.HEARTBEAT:
                var heartbeat = (mavlink_heartbeat_t)msg.data;
                Debug.Log($"Heartbeat: Type={(MAV_TYPE)heartbeat.type}, Mode={(MAV_MODE)heartbeat.base_mode}");
                break;

            // Добавьте другие сообщения по необходимости
        }
    }

    void Update()
    {
        // Пример: вращение 3D-модели дрона по данным ATTITUDE
        transform.rotation = Quaternion.Euler(
            -pitch * Mathf.Rad2Deg,
            yaw * Mathf.Rad2Deg,
            -roll * Mathf.Rad2Deg
        );
    }
}
