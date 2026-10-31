using Logistics.Wms.Domain;

namespace Logistics.Wms.Web.Components;

public static class Fmt
{
    public static string Ago(DateTime? at, DateTime now)
    {
        if (at is null) return "—";
        var s = (now - at.Value).TotalSeconds;
        var L = Services.Localizer.Current;
        string T(string en, double v) => L is null ? string.Format(en, v) : L.F(en, v);
        return s < 2 ? (L?["now"] ?? "now") : s < 60 ? T("{0:F0} s ago", s) : s < 3600 ? T("{0:F0} min ago", s / 60) : s < 86400 ? T("{0:F1} h ago", s / 3600) : at.Value.ToLocalTime().ToString("yyyy-MM-dd HH:mm");
    }

    public static string Time(DateTime? at) => at is null ? "—" : at.Value.ToLocalTime().ToString("HH:mm:ss");

    public static string Bytes(long chars) => chars < 1024 ? $"{chars} B" : chars < 1024 * 1024 ? $"{chars / 1024.0:F0} KB" : $"{chars / 1024.0 / 1024.0:F1} MB";

    /// <summary>"AtOperator" → "At operator".</summary>
    public static string Words(this Enum value)
    {
        var s = value.ToString();
        var sb = new System.Text.StringBuilder(s.Length + 4);
        for (var i = 0; i < s.Length; i++)
        {
            if (i > 0 && char.IsUpper(s[i]) && char.IsLower(s[i - 1])) sb.Append(' ').Append(char.ToLowerInvariant(s[i]));
            else sb.Append(s[i]);
        }
        return sb.ToString();
    }

    public static string Location(BinLocation l) => Services.Localizer.Current?[LocationEnglish(l)] ?? LocationEnglish(l);

    private static string LocationEnglish(BinLocation l) => l switch
    {
        BinLocation.Rack => "Rack",
        BinLocation.PickInbound => "Picking loop · inbound",
        BinLocation.AtStation => "At picking stop",
        BinLocation.PickReturn => "Picking loop · return",
        BinLocation.RestockStation => "At restock operator",
        BinLocation.RestockReturn => "Restock loop · return",
        _ => l.ToString(),
    };
}
