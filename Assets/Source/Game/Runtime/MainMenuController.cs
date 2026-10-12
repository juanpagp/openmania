using Tmnf.Game.Pure;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace Tmnf.Game
{
    /// <summary>
    /// The main menu scene controller: the title screen (Play / Quit) and
    /// the track selection list. The list is generated from the
    /// <see cref="TrackCatalog"/> asset — never hardcoded — showing each
    /// track's name, author, author time and the player's personal best.
    /// Selecting an entry hands the scene path to the Race scene.
    /// </summary>
    public sealed class MainMenuController : MonoBehaviour
    {
        private const string RaceScene = "Assets/Scenes/Race.unity";

        private RectTransform menuPanel;
        private RectTransform selectPanel;
        private RectTransform listContent;

        private void Start()
        {
            UiBuilder.EnsureEventSystem();
            Canvas canvas = UiBuilder.CreateCanvas(transform, "MainMenuCanvas");
            BuildMenuPanel(canvas.transform);
            BuildSelectPanel(canvas.transform);
            ShowSelect(GameSession.showTrackSelect);
            GameSession.showTrackSelect = false;
        }

        // ------------------------------------------------------------------
        // panels
        // ------------------------------------------------------------------

        private void BuildMenuPanel(Transform parent)
        {
            menuPanel = UiBuilder.Panel(parent, "MenuPanel",
                Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero,
                new Color(0.07f, 0.08f, 0.11f, 0.0f));
            UiBuilder.Label(menuPanel, "Title",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-600f, 120f), new Vector2(600f, 320f),
                "OPENMANIA", 110, UiBuilder.HexColor("#ffd866"), TextAnchor.MiddleCenter);
            UiBuilder.Label(menuPanel, "Subtitle",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-600f, 60f), new Vector2(600f, 120f),
                "a TrackMania Nations Forever physics sandbox", 24,
                new Color(1f, 1f, 1f, 0.6f), TextAnchor.MiddleCenter);

            UiBuilder.Button(menuPanel, "Play", new Vector2(0f, -30f),
                new Vector2(340f, 64f), "Play", 30, () => ShowSelect(true));
            UiBuilder.Button(menuPanel, "Quit", new Vector2(0f, -110f),
                new Vector2(340f, 64f), "Quit", 30, Quit);

            UiBuilder.Label(menuPanel, "Footer",
                new Vector2(0.5f, 0f), new Vector2(0.5f, 0f),
                new Vector2(-500f, 18f), new Vector2(500f, 60f),
                "vehicle physics: TMNF 2.11.26 (ported, bit-exact golden traces)",
                17, new Color(1f, 1f, 1f, 0.35f), TextAnchor.MiddleCenter);
        }

        private void BuildSelectPanel(Transform parent)
        {
            selectPanel = UiBuilder.Panel(parent, "SelectPanel",
                Vector2.zero, Vector2.one, Vector2.zero, Vector2.zero,
                new Color(0.07f, 0.08f, 0.11f, 0.0f));
            UiBuilder.Label(selectPanel, "Title",
                new Vector2(0.5f, 1f), new Vector2(0.5f, 1f),
                new Vector2(-500f, -96f), new Vector2(500f, -36f),
                "SELECT A TRACK", 44, Color.white, TextAnchor.MiddleCenter);

            UiBuilder.Button(selectPanel, "Back", new Vector2(0f, -350f),
                new Vector2(260f, 52f), "Back", 24, () => ShowSelect(false));

            // The list: a scroll view with one row per catalog entry.
            RectTransform view = UiBuilder.Panel(selectPanel, "ScrollView",
                new Vector2(0.5f, 0.5f), new Vector2(0.5f, 0.5f),
                new Vector2(-460f, -280f), new Vector2(460f, 170f),
                new Color(0f, 0f, 0f, 0.25f));
            ScrollRect scroll = view.gameObject.AddComponent<ScrollRect>();
            scroll.horizontal = false;
            scroll.vertical = true;
            scroll.scrollSensitivity = 40f;
            view.gameObject.AddComponent<RectMask2D>();
            Image viewportImage = view.GetComponent<Image>();
            viewportImage.color = new Color(0f, 0f, 0f, 0.2f);
            viewportImage.raycastTarget = true;

            RectTransform content = new GameObject("Content").AddComponent<RectTransform>();
            content.SetParent(view, false);
            content.anchorMin = new Vector2(0f, 1f);
            content.anchorMax = new Vector2(1f, 1f);
            content.pivot = new Vector2(0.5f, 1f);
            content.anchoredPosition = Vector2.zero;
            content.sizeDelta = new Vector2(0f, 0f);
            VerticalLayoutGroup layout = content.gameObject.AddComponent<VerticalLayoutGroup>();
            layout.childAlignment = TextAnchor.UpperCenter;
            layout.childForceExpandHeight = false;
            layout.childForceExpandWidth = true;
            layout.childControlHeight = true;
            layout.childControlWidth = true;
            layout.spacing = 10f;
            ContentSizeFitter fitter = content.gameObject.AddComponent<ContentSizeFitter>();
            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            scroll.content = content;
            scroll.viewport = view;
            listContent = content;
        }

        private void ShowSelect(bool visible)
        {
            menuPanel.gameObject.SetActive(!visible);
            selectPanel.gameObject.SetActive(visible);
            if (visible)
            {
                RebuildList();
            }
        }

        // ------------------------------------------------------------------
        // list
        // ------------------------------------------------------------------

        private void RebuildList()
        {
            for (int i = listContent.childCount - 1; i >= 0; i--)
            {
                Destroy(listContent.GetChild(i).gameObject);
            }

            TrackCatalog catalog = Resources.Load<TrackCatalog>("TrackCatalog");
            if (catalog == null)
            {
                AddMessageRow("No TrackCatalog found.\nCreate one via " +
                    "Create > OpenMania > Track Catalog and save it as " +
                    "Assets/Resources/TrackCatalog.asset");
                return;
            }
            if (catalog.tracks.Count == 0)
            {
                AddMessageRow("The track catalog is empty.\n" +
                    "Add your track scenes to Assets/Resources/TrackCatalog.asset " +
                    "(see the README).");
                return;
            }

            foreach (TrackCatalog.TrackEntry entry in catalog.tracks)
            {
                if (!entry.HasScene)
                {
                    continue;
                }
                string scenePath = entry.scenePath;
                string name = string.IsNullOrWhiteSpace(entry.displayName)
                    ? "Unnamed Track"
                    : entry.displayName;
                string author = string.IsNullOrWhiteSpace(entry.author)
                    ? "Unknown"
                    : entry.author;
                int personalBest = PersonalBestStore.Get(scenePath);

                string line2 = $"by {author}";
                if (entry.authorTimeMs > 0)
                {
                    line2 += $"    author {RaceTimeFormat.Time(entry.authorTimeMs)}";
                }
                if (personalBest > 0)
                {
                    line2 += $"    your best {RaceTimeFormat.Time(personalBest)}";
                }

                Button row = CreateListRow(listContent, 92f);
                row.onClick.AddListener(() => StartRace(scenePath, name));
                RectTransform rect = row.GetComponent<RectTransform>();

                Text title = UiBuilder.Label(rect, "Name",
                    Vector2.zero, Vector2.one, new Vector2(20f, 40f), new Vector2(-20f, -12f),
                    name, 28, Color.white, TextAnchor.MiddleLeft);
                title.raycastTarget = false;
                Text subtitle = UiBuilder.Label(rect, "Info",
                    Vector2.zero, Vector2.one, new Vector2(20f, 12f), new Vector2(-20f, 44f),
                    line2, 20, new Color(1f, 1f, 1f, 0.62f), TextAnchor.MiddleLeft);
                subtitle.raycastTarget = false;
            }
        }

        private void AddMessageRow(string message)
        {
            Button row = CreateListRow(listContent, 92f);
            row.interactable = false;
            RectTransform rect = row.GetComponent<RectTransform>();
            Text text = UiBuilder.Label(rect, "Message",
                Vector2.zero, Vector2.one, new Vector2(20f, 10f), new Vector2(-20f, -10f),
                message, 20, new Color(1f, 1f, 1f, 0.8f), TextAnchor.MiddleLeft);
            text.raycastTarget = false;
        }

        private static Button CreateListRow(Transform parent, float height)
        {
            GameObject go = new GameObject("TrackRow");
            go.transform.SetParent(parent, false);
            RectTransform rect = go.AddComponent<RectTransform>();
            LayoutElement element = go.AddComponent<LayoutElement>();
            element.minHeight = height;
            element.preferredHeight = height;
            Image image = go.AddComponent<Image>();
            image.color = new Color(0.16f, 0.18f, 0.24f, 0.95f);
            Button button = go.AddComponent<Button>();
            button.targetGraphic = image;
            ColorBlock colors = button.colors;
            colors.highlightedColor = new Color(1.2f, 1.2f, 1.2f, 1f);
            colors.pressedColor = new Color(0.7f, 0.7f, 0.7f, 1f);
            colors.fadeDuration = 0.08f;
            button.colors = colors;
            return button;
        }

        // ------------------------------------------------------------------
        // actions
        // ------------------------------------------------------------------

        private static void StartRace(string scenePath, string displayName)
        {
            if (string.IsNullOrEmpty(scenePath))
            {
                Debug.LogError("[Tmnf.Game] the catalog entry has no scene path.");
                return;
            }
            GameSession.selectedTrackScene = scenePath;
            GameSession.selectedTrackName = displayName;
            GameSession.lastTrackScene = scenePath;
            SceneManager.LoadScene(RaceScene);
        }

        private static void Quit()
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.isPlaying = false;
#else
            Application.Quit();
#endif
        }
    }
}
