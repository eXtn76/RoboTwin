using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace RoboTwin.Programs
{
    /// <summary>Ein Punkt eines Programms: Armhaltung (Gelenkwinkel) + Geschwindigkeit + Wartezeit.</summary>
    public class ProgramPoint
    {
        public string Name;
        public float[] Joints;   // Grad, URDF-Konvention
        public float Speed;      // Anteil der Maximalgeschwindigkeit, (0, 1]
        public float Wait;       // Sekunden nach Ankunft

        public ProgramPoint(string name, float[] joints, float speed = 1f, float wait = 0f)
        {
            Name = name;
            Joints = (float[])joints.Clone();
            Speed = speed;
            Wait = wait;
        }
    }

    /// <summary>Ein Programm: Name + Punkte in Abfahrreihenfolge.</summary>
    public class RobotProgram
    {
        public string Name;
        public readonly List<ProgramPoint> Points = new List<ProgramPoint>();

        public RobotProgram(string name = "Neues Programm")
        {
            Name = name;
        }
    }

    public class ProgramFormatException : Exception
    {
        public ProgramFormatException(string message) : base(message)
        {
        }
    }

    /// <summary>
    /// Liest und schreibt Programmdateien (docs/PROGRAM_FORMAT.md).
    /// Gegenstück zu controller/robotwin_controller/program.py.
    /// </summary>
    public static class ProgramFile
    {
        public const string Format = "robotwin-program";
        public const int Version = 1;
        public const string Extension = ".json";

        static readonly UTF8Encoding Utf8 = new UTF8Encoding(false);   // ohne BOM

        public static void Save(RobotProgram program, string path) => File.WriteAllText(path, ToJson(program), Utf8);

        public static RobotProgram Load(string path, int jointCount) => FromJson(File.ReadAllText(path, Utf8), jointCount);

        // ---- Schreiben ----

        /// <summary>Kopfdaten eingerückt, jeder Punkt auf EINER Zeile – so zeigt ein Git-Diff genau den geänderten Punkt.</summary>
        public static string ToJson(RobotProgram program)
        {
            var text = new StringBuilder();
            text.Append("{\n");
            text.Append($"  \"format\": \"{Format}\",\n");
            text.Append($"  \"version\": {Version},\n");
            text.Append($"  \"name\": {JsonConvert.ToString(program.Name)},\n");   // ToString: korrekt maskiert
            text.Append("  \"points\": [");

            for (int i = 0; i < program.Points.Count; i++)
            {
                text.Append(i == 0 ? "\n" : ",\n");
                text.Append("    ").Append(PointToJson(program.Points[i]).ToString(Formatting.None));
            }

            text.Append(program.Points.Count > 0 ? "\n  ]\n}\n" : "]\n}\n");
            return text.ToString();
        }

        static JObject PointToJson(ProgramPoint point)
        {
            var joints = new JArray();
            foreach (float angle in point.Joints)
                joints.Add(Round(angle));

            return new JObject
            {
                ["name"] = point.Name,
                ["joints"] = joints,
                ["speed"] = Round(point.Speed),
                ["wait"] = Round(point.Wait)
            };
        }

        // 0,001° genügt; ohne Runden stünden Werte wie 35.19999694824219 in der Datei.
        static double Round(float value) => Math.Round(value, 3);

        // ---- Lesen ----

        public static RobotProgram FromJson(string json, int jointCount)
        {
            JObject root;
            try
            {
                root = JObject.Parse(json);
            }
            catch (JsonException error)
            {
                throw new ProgramFormatException($"Kein gültiges JSON: {error.Message}");
            }

            if ((string)root["format"] != Format)
                throw new ProgramFormatException($"Feld 'format' muss \"{Format}\" sein.");

            JToken version = root["version"];
            if (version == null || version.Type != JTokenType.Integer || (int)version != Version)
                throw new ProgramFormatException($"Formatversion {version} wird nicht unterstützt (erwartet {Version}).");

            if (root["name"]?.Type != JTokenType.String)
                throw new ProgramFormatException("Feld 'name' fehlt oder ist kein Text.");

            if (!(root["points"] is JArray points))
                throw new ProgramFormatException("Feld 'points' fehlt oder ist keine Liste.");

            var program = new RobotProgram((string)root["name"]);
            for (int i = 0; i < points.Count; i++)
                program.Points.Add(ParsePoint($"Punkt {i + 1}", points[i], jointCount));
            return program;
        }

        static ProgramPoint ParsePoint(string where, JToken token, int jointCount)
        {
            if (!(token is JObject point))
                throw new ProgramFormatException($"{where}: muss ein Objekt sein.");

            if (point["name"]?.Type != JTokenType.String || string.IsNullOrWhiteSpace((string)point["name"]))
                throw new ProgramFormatException($"{where}: 'name' fehlt oder ist leer.");
            string name = (string)point["name"];
            where = $"{where} ({name})";

            if (!(point["joints"] is JArray joints) || joints.Count != jointCount || !joints.All(IsNumber))
                throw new ProgramFormatException($"{where}: 'joints' braucht {jointCount} Zahlen.");

            var angles = new float[jointCount];
            for (int i = 0; i < jointCount; i++)
                angles[i] = (float)joints[i];

            float speed = ReadNumber(point, "speed", 1f, where);
            if (!(speed > 0f && speed <= 1f))   // so formuliert, dass auch NaN scheitert
                throw new ProgramFormatException($"{where}: 'speed' muss in (0, 1] liegen.");

            float wait = ReadNumber(point, "wait", 0f, where);
            if (!(wait >= 0f) || float.IsInfinity(wait))
                throw new ProgramFormatException($"{where}: 'wait' muss ≥ 0 sein.");

            return new ProgramPoint(name, angles, speed, wait);
        }

        static float ReadNumber(JObject point, string field, float fallback, string where)
        {
            JToken token = point[field];
            if (token == null)
                return fallback;   // optionales Feld
            if (!IsNumber(token))
                throw new ProgramFormatException($"{where}: '{field}' muss eine Zahl sein.");
            return (float)token;
        }

        static bool IsNumber(JToken token) => token.Type == JTokenType.Integer || token.Type == JTokenType.Float;
    }
}
