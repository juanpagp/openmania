using Tmnf.Game.Pure;
using UnityEngine;
using UnityEngine.UI;

namespace Tmnf.Game
{
    /// <summary>
    /// The in-race interface: race time, live delta against the reference
    /// (personal best, else author time), checkpoint progress, speed,
    /// countdown, results screen, pause menu and the invalid-track report.
    /// Built entirely at runtime; the RaceRunner drives it through simple
    /// state setters so the game logic stays UI-free.
    /// </summary>
    public sealed class RaceHud : MonoBehaviour
    {
        private Text timeLabel;
        private Text deltaLabel;
        private Text progressLabel;
        private Text speedLabel;
        private Text countdownLabel;
        private Text trackLabel;
        private Text hintLabel;
        private Text debugLabel;

        private RectTransform resultsPanel;
        private Text resultsTitle;
        private Text resultsBody;
        private RectTransform pausePanel;
        private RectTransform errorPanel;
        private Text errorBody;

        public System.Action onRetry;
        public System.Action onResume;
        public System.Action onRestartFromPause;
        public System.Action onTrackSelect;
        public System.Action onMainMenu;
        public System.Action onErrorBack;

        public void Build(string trackName, string author, int authorTimeMs, int personalBestMs)
        {
            Canvas canvas = UiBuilder.CreateCanvas(transform, "RaceHudCanvas");

            // -- top bar: time + delta + track info --
            UiBuilder.Panel(canvas.transform, "TopBar",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-420f, -116f), new Vector2(420f, -12f),
                new Color(0f, 0f, 0f, 0.45f));
            timeLabel = UiBuilder.Label(canvas.transform, "Time",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-400f, -84f), new Vector2(400f, -24f),
                "00:00.000", 44, Color.white, TextAnchor.MiddleCenter);
            deltaLabel = UiBuilder.Label(canvas.transform, "Delta",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-400f, -114f), new Vector2(400f, -84f),
                "", 24, Color.gray, TextAnchor.MiddleCenter);
            trackLabel = UiBuilder.Label(canvas.transform, "Track",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-400f, -44f), new Vector2(400f, -14f),
                "", 18, new Color(0.8f, 0.85f, 0.9f), TextAnchor.MiddleCenter);

            string subtitle = $"by {author}";
            if (authorTimeMs > 0)
            {
                subtitle += $"   -   author time {RaceTimeFormat.Time(authorTimeMs)}";
            }
            trackLabel.text = $"{trackName}  {subtitle}";

            // -- checkpoint progress + speed, top left --
            progressLabel = UiBuilder.Label(canvas.transform, "Progress",
                new Vector2(0f, 1f), new Vector2(0f, 1f),
                new Vector2(16f, -60f), new Vector2(320f, -20f),
                "", 26, Color.white, TextAnchor.MiddleLeft);
            speedLabel = UiBuilder.Label(canvas.transform, "Speed",
                new Vector2(0f, 1f), new Vector2(0f, 1f),
                new Vector2(16f, -100f), new Vector2(320f, -62f),
                "", 22, new Color(0.8f, 0.85f, 0.9f), TextAnchor.MiddleLeft);

            // -- controls hint, bottom left --
            hintLabel = UiBuilder.Label(canvas.transform, "Hint",
                new Vector2(0f, 0f), new Vector2(0f, 0f),
                new Vector2(16f, 16f), new Vector2(620f, 76f),
                "Drive: WASD / arrows    Restart: R    Pause: Esc\n" +
                "F3: collision debug    V: toggle car visuals",
                17, new Color(1f, 1f, 1f, 0.55f), TextAnchor.LowerLeft);

            // -- debug state, bottom right --
            debugLabel = UiBuilder.Label(canvas.transform, "Debug",
                new Vector2(1f, 0f), new Vector2(1f, 0f),
                new Vector2(-320f, 16f), new Vector2(-16f, 48f),
                "", 17, new Color(1f, 1f, 1f, 0.55f), TextAnchor.LowerRight);

            // -- countdown, center --
            countdownLabel = UiBuilder.Label(canvas.transform, "Countdown",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-400f, 40f), new Vector2(400f, 240f),
                "", 140, Color.white, TextAnchor.MiddleCenter);

            BuildResultsPanel(canvas.transform);
            BuildPausePanel(canvas.transform);
            BuildErrorPanel(canvas.transform);
            SetCountdown("");
            SetProgress(0, 0);
            SetSpeed(0f);
            SetDebug(false, true);
        }

        // ------------------------------------------------------------------
        // live setters
        // ------------------------------------------------------------------

        public void SetTime(int milliseconds, int referenceMs, bool hasReference)
        {
            timeLabel.text = RaceTimeFormat.Time(milliseconds);
            if (hasReference)
            {
                int delta = milliseconds - referenceMs;
                deltaLabel.text = RaceTimeFormat.Delta(delta);
                deltaLabel.color = delta <= 0
                    ? UiBuilder.HexColor("#7dff8a")
                    : UiBuilder.HexColor("#ff7d7d");
            }
            else
            {
                deltaLabel.text = "";
            }
        }

        public void SetProgress(int collected, int total)
        {
            progressLabel.text = total > 0 ? $"Checkpoints {collected}/{total}" : "";
        }

        public void SetSpeed(float metersPerSecond)
        {
            speedLabel.text = $"{(int)(metersPerSecond * 3.6f)} km/h";
        }

        public void SetCountdown(string text)
        {
            countdownLabel.text = text;
            countdownLabel.enabled = !string.IsNullOrEmpty(text);
        }

        public void SetDebug(bool visible, bool visualsVisible)
        {
            debugLabel.text = $"F3 collision debug: {(visible ? "ON" : "off")}   " +
                $"car visuals (V): {(visualsVisible ? "ON" : "off")}";
        }

        // ------------------------------------------------------------------
        // results
        // ------------------------------------------------------------------

        private void BuildResultsPanel(Transform parent)
        {
            resultsPanel = UiBuilder.Panel(parent, "Results",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-430f, -250f), new Vector2(430f, 250f),
                new Color(0.05f, 0.06f, 0.09f, 0.94f));
            resultsTitle = UiBuilder.Label(resultsPanel, "Title",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-400f, -70f), new Vector2(400f, -18f),
                "FINISHED", 42, UiBuilder.HexColor("#ffd866"), TextAnchor.MiddleCenter);
            resultsBody = UiBuilder.Label(resultsPanel, "Body",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-400f, -200f), new Vector2(400f, -80f),
                "", 25, Color.white, TextAnchor.UpperCenter);

            UiBuilder.Button(resultsPanel, "Retry", new Vector2(-220f, -130f),
                new Vector2(200f, 52f), "Retry", 24, () => onRetry?.Invoke());
            UiBuilder.Button(resultsPanel, "TrackSelect", new Vector2(0f, -130f),
                new Vector2(200f, 52f), "Track Select", 24, () => onTrackSelect?.Invoke());
            UiBuilder.Button(resultsPanel, "Menu", new Vector2(220f, -130f),
                new Vector2(200f, 52f), "Main Menu", 24, () => onMainMenu?.Invoke());
            resultsPanel.gameObject.SetActive(false);
        }

        public void ShowResults(
            int timeMs, int personalBestBefore, int personalBestAfter,
            int authorTimeMs, string trackName)
        {
            string body = $"Time           {RaceTimeFormat.Time(timeMs)}\n";
            if (personalBestAfter == timeMs && personalBestBefore != timeMs)
            {
                body += $"Personal best  {RaceTimeFormat.Time(personalBestAfter)}  (new best!)\n";
            }
            else
            {
                body += $"Personal best  {RaceTimeFormat.Time(personalBestAfter)}\n";
            }
            if (authorTimeMs > 0)
            {
                body += $"Author time    {RaceTimeFormat.Time(authorTimeMs)}\n";
                int reference = personalBestBefore > 0 ? personalBestBefore : authorTimeMs;
                body += $"Delta          {RaceTimeFormat.Delta(timeMs - reference)}\n";
            }
            else if (personalBestBefore > 0)
            {
                body += $"Delta          {RaceTimeFormat.Delta(timeMs - personalBestBefore)}\n";
            }
            resultsBody.text = body;
            resultsPanel.gameObject.SetActive(true);
        }

        public void HideResults()
        {
            resultsPanel.gameObject.SetActive(false);
        }

        // ------------------------------------------------------------------
        // pause
        // ------------------------------------------------------------------

        private void BuildPausePanel(Transform parent)
        {
            pausePanel = UiBuilder.Panel(parent, "Pause",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-330f, -210f), new Vector2(330f, 210f),
                new Color(0.05f, 0.06f, 0.09f, 0.94f));
            UiBuilder.Label(pausePanel, "Title",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-300f, -64f), new Vector2(300f, -20f),
                "PAUSED", 38, Color.white, TextAnchor.MiddleCenter);
            UiBuilder.Button(pausePanel, "Resume", new Vector2(0f, 20f),
                new Vector2(260f, 48f), "Resume", 22, () => onResume?.Invoke());
            UiBuilder.Button(pausePanel, "Restart", new Vector2(0f, -40f),
                new Vector2(260f, 48f), "Restart Run", 22, () => onRestartFromPause?.Invoke());
            UiBuilder.Button(pausePanel, "TrackSelect", new Vector2(0f, -100f),
                new Vector2(260f, 48f), "Track Select", 22, () => onTrackSelect?.Invoke());
            UiBuilder.Button(pausePanel, "Menu", new Vector2(0f, -160f),
                new Vector2(260f, 48f), "Main Menu", 22, () => onMainMenu?.Invoke());
            pausePanel.gameObject.SetActive(false);
        }

        public void ShowPause()
        {
            pausePanel.gameObject.SetActive(true);
        }

        public void HidePause()
        {
            pausePanel.gameObject.SetActive(false);
        }

        // ------------------------------------------------------------------
        // errors
        // ------------------------------------------------------------------

        private void BuildErrorPanel(Transform parent)
        {
            errorPanel = UiBuilder.Panel(parent, "Errors",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-540f, -320f), new Vector2(540f, 320f),
                new Color(0.13f, 0.03f, 0.03f, 0.95f));
            UiBuilder.Label(errorPanel, "Title",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-500f, -70f), new Vector2(500f, -20f),
                "INVALID TRACK", 38, UiBuilder.HexColor("#ff8d8d"), TextAnchor.MiddleCenter);
            errorBody = UiBuilder.Label(errorPanel, "Body",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-500f, -240f), new Vector2(500f, -90f),
                "", 20, Color.white, TextAnchor.UpperLeft);
            UiBuilder.Button(errorPanel, "Back", new Vector2(0f, -200f),
                new Vector2(220f, 48f), "Back to Track Select", 20, () => onErrorBack?.Invoke());
            errorPanel.gameObject.SetActive(false);
        }

        public void ShowErrors(System.Collections.Generic.List<string> errors)
        {
            string body = "";
            int shown = 0;
            foreach (string error in errors)
            {
                if (shown++ >= 12)
                {
                    body += $"... and {errors.Count - shown + 1} more\n";
                    break;
                }
                body += "- " + error + "\n";
            }
            errorBody.text = body;
            errorPanel.gameObject.SetActive(true);
        }
    }
}
