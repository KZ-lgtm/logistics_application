using System.Globalization;
using System.Text.RegularExpressions;
using Logistics.Wms.Web.Components;
using Microsoft.JSInterop;

namespace Logistics.Wms.Web.Services;

/// <summary>
/// UI language of the browser edition: English or Traditional Chinese (Taiwan). Texts are looked up by their English wording
/// (<c>L["Orders"]</c>, <c>L.F("Order {0} created.", id)</c>); anything without a translation stays English. Business-rule messages
/// of the WMS services are translated by pattern (<see cref="Message"/>). The choice is remembered in this browser; the first visit
/// follows the browser's language.
/// </summary>
public sealed class Localizer(IJSRuntime js)
{
    public const string English = "en", Chinese = "zh-Hant";
    private const string StorageKey = "logistics-wms/lang";

    /// <summary>For static helpers (formatting) and components without injection.</summary>
    public static Localizer? Current { get; private set; }

    public string Language { get; private set; } = English;
    public bool IsChinese => Language == Chinese;
    public event Action? Changed;

    public string this[string english] => IsChinese && Strings.ZhHant.TryGetValue(Normalize(english), out var zh) ? zh : english;

    private static readonly Dictionary<string, string> Normalized = [];

    /// <summary>Keys may be wrapped over several lines in the markup: whitespace runs count as one space.</summary>
    private static string Normalize(string s)
    {
        if (s.IndexOf('\n') < 0 && s.IndexOf("  ", StringComparison.Ordinal) < 0) return s;
        if (!Normalized.TryGetValue(s, out var n)) Normalized[s] = n = Regex.Replace(s.Trim(), @"\s+", " ");
        return n;
    }

    public string F(string english, params object?[] args) => string.Format(CultureInfo.InvariantCulture, this[english], args);

    /// <summary>Display name of an enum value ("AtOperator" → "At operator" / "在包裝作業員").</summary>
    public string Enum(Enum value)
    {
        var key = value.GetType().Name + "." + value;
        return IsChinese && Strings.ZhHantEnums.TryGetValue(key, out var zh) ? zh : value.Words();
    }

    /// <summary>
    /// A text from the WMS services (business-rule message, task description, event text, ledger reference) in the UI language:
    /// exact texts from the table, otherwise the first matching pattern; ${name} inserts a group (enum names translated),
    /// ${name:m} translates the group as a text of its own.
    /// </summary>
    public string Message(string? english)
    {
        if (!IsChinese || string.IsNullOrEmpty(english)) return english ?? "";
        if (Strings.ZhHant.TryGetValue(english, out var exact)) return exact;
        foreach (var (pattern, zh) in Strings.ZhHantMessages)
        {
            var m = pattern.Match(english);
            if (!m.Success) continue;
            return Regex.Replace(zh, @"\$\{(\w+)(:m)?\}", g =>
            {
                var v = m.Groups[g.Groups[1].Value].Value;
                return g.Groups[2].Success ? Message(v) : TranslateToken(v);
            });
        }
        return english;
    }

    /// <summary>Enum names and a few nouns inside messages ("is Completed", "Pick task") are translated too.</summary>
    private static string TranslateToken(string token)
    {
        foreach (var (key, zh) in Strings.ZhHantEnums)
            if (key.EndsWith("." + token, StringComparison.Ordinal)) return zh;
        return token;
    }

    public async Task InitializeAsync()
    {
        Current = this;
        string? saved = null;
        try { saved = await js.InvokeAsync<string?>("wms.language", StorageKey); } catch (JSException) { }
        Language = saved == Chinese || saved == English ? saved : English;
        ApplyCulture();
        try { await js.InvokeVoidAsync("wms.setLanguage", StorageKey, Language); } catch (JSException) { }
    }

    public async Task SetAsync(string language)
    {
        if (language == Language) return;
        Language = language == Chinese ? Chinese : English;
        ApplyCulture();
        try { await js.InvokeVoidAsync("wms.setLanguage", StorageKey, Language); } catch (JSException) { }
        Changed?.Invoke();
    }

    private void ApplyCulture() => CultureInfo.DefaultThreadCurrentUICulture = CultureInfo.CurrentUICulture = CultureInfo.InvariantCulture;
}
