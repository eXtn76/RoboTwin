using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading;

namespace RoboTwin.Connection
{
    /// <summary>
    /// Verbindung zum Python-Controller über TCP (docs/PROTOCOL.md).
    ///
    /// Zwei Threads:
    /// * Netzwerk-Thread: verbindet und liest Zeilen (blockierend) → legt sie in die Warteschlange 'inbox'.
    /// * Haupt-Thread (Unity): Tick() holt die Zeilen ab, übersetzt sie und löst die Events aus.
    /// Der Netzwerk-Thread fasst NIE Unity-Objekte oder Events an – nur die thread-sichere Warteschlange.
    /// </summary>
    public class TcpConnection : IRobotConnection
    {
        const double WatchdogSeconds = 1.0;          // so lange ohne Nachricht → Verbindung gilt als verloren
        const double ReconnectDelaySeconds = 2.0;    // Abstand zwischen Verbindungsversuchen
        const int ConnectTimeoutMilliseconds = 2000;
        const int WriteTimeoutMilliseconds = 1000;
        static readonly UTF8Encoding Utf8 = new UTF8Encoding(false);   // UTF-8 ohne BOM

        readonly string host;
        readonly int port;
        readonly string[] expectedNames;   // Grenzen aus Unity – zum Vergleich mit "hello"
        readonly float[] expectedMin;
        readonly float[] expectedMax;

        // ---- Übergabe Netzwerk-Thread → Haupt-Thread ----
        readonly ConcurrentQueue<NetworkEvent> inbox = new ConcurrentQueue<NetworkEvent>();

        // Schützt client/stream/generation, auf die BEIDE Threads zugreifen.
        readonly object socketLock = new object();
        TcpClient client;
        NetworkStream stream;
        int generation;   // zählt Verbindungsversuche – Ereignisse alter Versuche werden verworfen

        // ---- Nur im Haupt-Thread benutzt ----
        ConnectionStatus status = ConnectionStatus.Disconnected;
        ControlRole role = ControlRole.Observer;
        bool wantsConnection;     // soll die Verbindung bestehen (bzw. wiederhergestellt werden)?
        bool helloPending;        // TCP steht, aber die Begrüßung "hello" fehlt noch
        int nextCommandId = 1;
        double clock;             // Sekunden, aus Tick(deltaTime) aufsummiert
        double lastMessageTime;
        double nextReconnectTime;

        // Events werden gesammelt und erst am Ende von Tick() ausgelöst (Regel aus IRobotConnection).
        bool statusChangePending;
        bool roleChangePending;
        readonly Queue<string> pendingWarnings = new Queue<string>();
        readonly Queue<CommandResult> localResults = new Queue<CommandResult>();

        public TcpConnection(string host, int port, string[] jointNames, float[] minAngles, float[] maxAngles)
        {
            this.host = host;
            this.port = port;
            expectedNames = (string[])jointNames.Clone();
            expectedMin = (float[])minAngles.Clone();
            expectedMax = (float[])maxAngles.Clone();
        }

        public string Name => "Netzwerk";
        public ConnectionStatus Status => status;
        public ControlRole Role => role;

        public event Action<ConnectionStatus> StatusChanged;
        public event Action<ControlRole> RoleChanged;
        public event Action<RobotState> StateReceived;
        public event Action<CommandResult> CommandResultReceived;
        public event Action<string> Warning;

        public void Connect()
        {
            wantsConnection = true;
            if (status == ConnectionStatus.Disconnected || status == ConnectionStatus.Failed)
                StartConnecting();
        }

        public void Disconnect()
        {
            wantsConnection = false;
            helloPending = false;
            CloseSocket();   // lässt auch den Netzwerk-Thread enden (ReadLine bricht ab)
            SetRole(ControlRole.Observer);
            SetStatus(ConnectionStatus.Disconnected);
        }

        public int SendCommand(RobotCommand command)
        {
            command.Id = nextCommandId++;

            if (status != ConnectionStatus.Connected)
            {
                localResults.Enqueue(CommandResult.Error(command.Id, "not_connected", "Nicht verbunden."));
                return command.Id;
            }

            byte[] data = Utf8.GetBytes(ProtocolCodec.Encode(command) + "\n");
            try
            {
                lock (socketLock)
                    stream.Write(data, 0, data.Length);   // kleine Nachricht, kehrt praktisch sofort zurück
            }
            catch (Exception error) when (error is IOException || error is ObjectDisposedException
                                          || error is InvalidOperationException)
            {
                localResults.Enqueue(CommandResult.Error(command.Id, "not_connected", "Senden fehlgeschlagen."));
                Fail($"Senden fehlgeschlagen: {error.Message}");
            }
            return command.Id;
        }

        /// <summary>Haupt-Thread, einmal pro Frame: Nachrichten abholen, Watchdog, Neuverbindung, Events.</summary>
        public void Tick(float deltaTime)
        {
            clock += deltaTime;

            // 1. Alles abholen, was der Netzwerk-Thread seit dem letzten Frame empfangen hat.
            while (inbox.TryDequeue(out NetworkEvent networkEvent))
            {
                if (networkEvent.Generation != generation)
                    continue;   // stammt von einem bereits beendeten Verbindungsversuch

                switch (networkEvent.Kind)
                {
                    case NetworkEventKind.Opened:
                        helloPending = true;          // jetzt muss "hello" kommen
                        lastMessageTime = clock;
                        break;
                    case NetworkEventKind.Line:
                        HandleLine(networkEvent.Text);
                        break;
                    case NetworkEventKind.Closed:
                        Fail(networkEvent.Text);
                        break;
                }
            }

            // 2. Watchdog: Der Controller sendet 50-mal pro Sekunde. Kommt 1 s lang nichts, ist die Verbindung tot –
            //    auch wenn TCP das (z. B. bei gezogenem Kabel) noch gar nicht bemerkt hat.
            if ((status == ConnectionStatus.Connected || helloPending) && clock - lastMessageTime > WatchdogSeconds)
                Fail(helloPending ? "Keine Begrüßung (hello) vom Controller." : "Seit über 1 s keine Daten vom Controller.");

            // 3. Automatisch neu verbinden, solange die Verbindung gewünscht ist.
            if (status == ConnectionStatus.Failed && wantsConnection && clock >= nextReconnectTime)
                StartConnecting();

            // 4. Gesammelte Events auslösen.
            if (statusChangePending)
            {
                statusChangePending = false;
                StatusChanged?.Invoke(status);
            }
            if (roleChangePending)
            {
                roleChangePending = false;
                RoleChanged?.Invoke(role);
            }
            while (pendingWarnings.Count > 0)
                Warning?.Invoke(pendingWarnings.Dequeue());
            while (localResults.Count > 0)
                CommandResultReceived?.Invoke(localResults.Dequeue());
        }

        // ---- Nachrichten verarbeiten (Haupt-Thread) ----

        void HandleLine(string line)
        {
            lastMessageTime = clock;

            ControllerMessage message;
            try
            {
                message = ProtocolCodec.Decode(line);
            }
            catch (ProtocolException error)
            {
                pendingWarnings.Enqueue(error.Message);   // eine kaputte Zeile beendet nicht die Verbindung
                return;
            }

            switch (message)
            {
                case HelloMessage hello:
                    HandleHello(hello);
                    break;
                case StateMessage state when status == ConnectionStatus.Connected:
                    StateReceived?.Invoke(state.State);
                    break;
                case ResultMessage result:
                    CommandResultReceived?.Invoke(result.Result);
                    break;
                case RoleMessage roleMessage:
                    SetRole(roleMessage.Role);
                    break;
                // null: unbekannter Nachrichtentyp → ignorieren (Vorwärtskompatibilität)
            }
        }

        void HandleHello(HelloMessage hello)
        {
            helloPending = false;

            if (hello.Protocol != ProtocolCodec.ProtocolVersion)
            {
                wantsConnection = false;   // erneute Versuche würden genauso scheitern
                Fail($"Controller spricht Protokoll {hello.Protocol}, Unity erwartet {ProtocolCodec.ProtocolVersion}.");
                return;
            }

            CheckJointLimits(hello.Joints);
            SetRole(hello.Role);
            SetStatus(ConnectionStatus.Connected);
        }

        // Stimmen die Grenzen des Controllers mit denen in Unity überein? (z. B. andere URDF)
        void CheckJointLimits(JointInfo[] joints)
        {
            if (joints.Length != expectedMin.Length)
            {
                pendingWarnings.Enqueue($"Controller meldet {joints.Length} Gelenke, Unity kennt {expectedMin.Length}.");
                return;
            }

            var differences = new List<string>();
            for (int i = 0; i < joints.Length; i++)
            {
                if (Math.Abs(joints[i].Min - expectedMin[i]) > 0.01f || Math.Abs(joints[i].Max - expectedMax[i]) > 0.01f)
                    differences.Add($"{expectedNames[i]}: Controller [{joints[i].Min:F0}°, {joints[i].Max:F0}°], " +
                                    $"Unity [{expectedMin[i]:F0}°, {expectedMax[i]:F0}°]");
            }
            if (differences.Count > 0)
                pendingWarnings.Enqueue("Achsgrenzen weichen ab – " + string.Join("; ", differences));
        }

        void Fail(string reason)
        {
            helloPending = false;
            CloseSocket();
            SetRole(ControlRole.Observer);
            SetStatus(ConnectionStatus.Failed);
            nextReconnectTime = clock + ReconnectDelaySeconds;
            pendingWarnings.Enqueue(wantsConnection ? $"{reason} Neuer Versuch in {ReconnectDelaySeconds:F0} s." : reason);
        }

        void SetStatus(ConnectionStatus newStatus)
        {
            if (status == newStatus)
                return;
            status = newStatus;
            statusChangePending = true;
        }

        void SetRole(ControlRole newRole)
        {
            if (role == newRole)
                return;
            role = newRole;
            roleChangePending = true;
        }

        // ---- Verbindungsaufbau (startet den Netzwerk-Thread) ----

        void StartConnecting()
        {
            CloseSocket();   // erhöht generation → Reste des alten Versuchs werden ignoriert
            int attempt;
            lock (socketLock)
                attempt = generation;

            SetStatus(ConnectionStatus.Connecting);

            // IsBackground: Der Thread hält Unity beim Beenden nicht fest.
            var thread = new Thread(() => ConnectAndReceive(attempt))
            {
                IsBackground = true,
                Name = "RoboTwin TCP"
            };
            thread.Start();
        }

        void CloseSocket()
        {
            lock (socketLock)
            {
                generation++;
                stream = null;
                client?.Close();   // ein blockiertes ReadLine im Netzwerk-Thread bricht dadurch ab
                client = null;
            }
        }

        // ---- Netzwerk-Thread ----

        void ConnectAndReceive(int attempt)
        {
            var newClient = new TcpClient();
            try
            {
                // Verbinden mit Zeitlimit: ConnectAsync starten und höchstens 2 s darauf warten.
                if (!newClient.ConnectAsync(host, port).Wait(ConnectTimeoutMilliseconds))
                    throw new TimeoutException($"Zeitüberschreitung beim Verbinden mit {host}:{port}.");

                newClient.NoDelay = true;   // kleine Nachrichten sofort senden (Nagle-Algorithmus aus)
                NetworkStream newStream = newClient.GetStream();
                newStream.WriteTimeout = WriteTimeoutMilliseconds;

                lock (socketLock)
                {
                    if (attempt != generation)   // inzwischen getrennt oder neu gestartet → aufgeben
                    {
                        newClient.Close();
                        return;
                    }
                    client = newClient;
                    stream = newStream;
                }
                inbox.Enqueue(NetworkEvent.Opened(attempt));

                var reader = new StreamReader(newStream, Utf8);
                string line;
                while ((line = reader.ReadLine()) != null)   // blockiert – aber nur diesen Thread
                    inbox.Enqueue(NetworkEvent.Line(attempt, line));

                inbox.Enqueue(NetworkEvent.Closed(attempt, "Der Controller hat die Verbindung beendet."));
            }
            catch (Exception error)
            {
                // Alles abfangen: Eine unbehandelte Exception in einem eigenen Thread könnte den
                // ganzen Prozess beenden. Stattdessen als Ereignis an den Haupt-Thread melden.
                newClient.Close();
                inbox.Enqueue(NetworkEvent.Closed(attempt, Describe(error)));
            }
        }

        string Describe(Exception error)
        {
            if (error is AggregateException aggregate && aggregate.InnerException != null)
                error = aggregate.InnerException;   // ConnectAsync verpackt Fehler in AggregateException

            return error switch
            {
                SocketException socket when socket.SocketErrorCode == SocketError.ConnectionRefused =>
                    $"Kein Controller unter {host}:{port} erreichbar.",
                TimeoutException timeout => timeout.Message,
                IOException _ => "Verbindung abgebrochen.",
                ObjectDisposedException _ => "Verbindung geschlossen.",
                _ => error.Message
            };
        }

        // ---- Ereignisse vom Netzwerk-Thread ----

        enum NetworkEventKind
        {
            Opened,   // TCP-Verbindung steht
            Line,     // eine Zeile empfangen
            Closed    // Verbindung beendet oder fehlgeschlagen (Text = Grund)
        }

        readonly struct NetworkEvent
        {
            public readonly NetworkEventKind Kind;
            public readonly int Generation;
            public readonly string Text;

            NetworkEvent(NetworkEventKind kind, int generation, string text)
            {
                Kind = kind;
                Generation = generation;
                Text = text;
            }

            public static NetworkEvent Opened(int generation) => new NetworkEvent(NetworkEventKind.Opened, generation, null);
            public static NetworkEvent Line(int generation, string text) => new NetworkEvent(NetworkEventKind.Line, generation, text);
            public static NetworkEvent Closed(int generation, string reason) => new NetworkEvent(NetworkEventKind.Closed, generation, reason);
        }
    }
}
