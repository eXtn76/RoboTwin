using System;
using System.Collections.Generic;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace RoboTwin.Connection
{
    // ---- Nachrichten vom Controller (nach dem Lesen) ----

    public abstract class ControllerMessage
    {
    }

    /// <summary>Grenzen eines Gelenks, wie sie der Controller in "hello" meldet.</summary>
    public readonly struct JointInfo
    {
        public readonly string Name;
        public readonly float Min;
        public readonly float Max;
        public readonly float MaxSpeed;

        public JointInfo(string name, float min, float max, float maxSpeed)
        {
            Name = name;
            Min = min;
            Max = max;
            MaxSpeed = maxSpeed;
        }
    }

    public sealed class HelloMessage : ControllerMessage
    {
        public readonly int Protocol;
        public readonly string ControllerName;
        public readonly ControlRole Role;
        public readonly int StateRate;
        public readonly JointInfo[] Joints;

        public HelloMessage(int protocol, string controllerName, ControlRole role, int stateRate, JointInfo[] joints)
        {
            Protocol = protocol;
            ControllerName = controllerName;
            Role = role;
            StateRate = stateRate;
            Joints = joints;
        }
    }

    public sealed class RoleMessage : ControllerMessage
    {
        public readonly ControlRole Role;
        public RoleMessage(ControlRole role) => Role = role;
    }

    public sealed class ResultMessage : ControllerMessage
    {
        public readonly CommandResult Result;
        public ResultMessage(CommandResult result) => Result = result;
    }

    public sealed class StateMessage : ControllerMessage
    {
        public readonly long Seq;
        public readonly RobotState State;

        public StateMessage(long seq, RobotState state)
        {
            Seq = seq;
            State = state;
        }
    }

    /// <summary>Eine Zeile vom Controller ließ sich nicht lesen.</summary>
    public class ProtocolException : Exception
    {
        public ProtocolException(string message) : base(message)
        {
        }
    }

    /// <summary>
    /// Übersetzt zwischen C#-Objekten und JSON-Zeilen (docs/PROTOCOL.md).
    /// Gegenstück zu controller/robotwin_controller/protocol.py.
    /// Newtonsoft schreibt und liest Zahlen IMMER mit Punkt – unabhängig von der Systemsprache.
    /// </summary>
    public static class ProtocolCodec
    {
        public const int ProtocolVersion = 1;

        // ---- Schreiben: Befehl → JSON-Zeile (ohne "\n") ----

        public static string Encode(RobotCommand command)
        {
            JObject json = command switch
            {
                MoveJointsCommand move => new JObject
                {
                    ["type"] = "move_joints",
                    ["id"] = move.Id,
                    ["joints"] = ToJsonArray(move.Joints),
                    ["speed"] = move.Speed
                },
                StopCommand stop => new JObject
                {
                    ["type"] = "stop",
                    ["id"] = stop.Id
                },
                SetOverrideCommand setOverride => new JObject
                {
                    ["type"] = "set_override",
                    ["id"] = setOverride.Id,
                    ["percent"] = setOverride.Percent
                },
                _ => throw new ArgumentException($"Unbekannter Befehl {command.GetType().Name}.")
            };
            return json.ToString(Formatting.None);   // kompakt, eine Zeile
        }

        // ---- Lesen: JSON-Zeile → Nachricht ----

        /// <summary>
        /// Liest eine Zeile vom Controller. Gibt null zurück bei unbekanntem Nachrichtentyp
        /// (Vorwärtskompatibilität: neuere Controller dürfen neue Nachrichten schicken).
        /// Wirft ProtocolException bei kaputten Zeilen.
        /// </summary>
        public static ControllerMessage Decode(string line)
        {
            JObject json;
            try
            {
                json = JObject.Parse(line);
            }
            catch (JsonException error)
            {
                throw new ProtocolException($"Ungültiges JSON vom Controller: {error.Message}");
            }

            try
            {
                switch ((string)json["type"])
                {
                    case "state":
                        return DecodeState(json);
                    case "ack":
                        return new ResultMessage(CommandResult.Ack((int)json["id"]));
                    case "error":
                        return new ResultMessage(CommandResult.Error(
                            ReadOptionalId(json), (string)json["code"] ?? "", (string)json["message"] ?? ""));
                    case "hello":
                        return DecodeHello(json);
                    case "role":
                        return new RoleMessage(ParseRole((string)json["role"]));
                    default:
                        return null;
                }
            }
            catch (Exception error) when (!(error is ProtocolException))
            {
                // Fehlendes Feld oder falscher Typ: (int)null, (float)"text" usw. werfen hier.
                throw new ProtocolException($"Unvollständige Nachricht vom Controller ({error.GetType().Name}): {line}");
            }
        }

        static StateMessage DecodeState(JObject json)
        {
            RobotStatus status = (string)json["status"] switch
            {
                "moving" => RobotStatus.Moving,
                "error" => RobotStatus.Error,
                _ => RobotStatus.Idle
            };
            var state = new RobotState(
                ReadFloatArray((JArray)json["joints"]),
                status,
                (float)json["override"],
                (double)json["timestamp"]);
            return new StateMessage((long)json["seq"], state);
        }

        static HelloMessage DecodeHello(JObject json)
        {
            var joints = new List<JointInfo>();
            foreach (JToken joint in (JArray)json["joints"])
            {
                joints.Add(new JointInfo(
                    (string)joint["name"],
                    (float)joint["min"],
                    (float)joint["max"],
                    (float)joint["max_speed"]));
            }

            return new HelloMessage(
                (int)json["protocol"],
                (string)json["controller"] ?? "?",
                ParseRole((string)json["role"]),
                (int?)json["state_rate"] ?? 0,
                joints.ToArray());
        }

        // Zahlenlisten von Hand übersetzen statt per JArray.FromObject/ToObject<float[]>():
        // Das kommt ohne Reflection aus – schneller und auch in IL2CPP-Builds (AOT) unproblematisch.
        static JArray ToJsonArray(float[] values)
        {
            var array = new JArray();
            foreach (float value in values)
                array.Add(value);
            return array;
        }

        static float[] ReadFloatArray(JArray array)
        {
            var values = new float[array.Count];
            for (int i = 0; i < values.Length; i++)
                values[i] = (float)array[i];
            return values;
        }

        // Bei kaputten Nachrichten schickt der Controller "id": null → hier 0 (gültige Ids sind ≥ 1).
        static int ReadOptionalId(JObject json)
        {
            JToken id = json["id"];
            return id != null && id.Type == JTokenType.Integer ? (int)id : 0;
        }

        static ControlRole ParseRole(string role) =>
            role == "owner" ? ControlRole.Owner : ControlRole.Observer;
    }
}
