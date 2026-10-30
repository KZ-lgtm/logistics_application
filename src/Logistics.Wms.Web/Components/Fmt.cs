using Logistics.Wms.Domain;

namespace Logistics.Wms.Web.Components;

public static class Fmt
{
    public static string Ago(DateTime? at, DateTime now)
    {
        if (at is null) return "—";
        var s = (now - at.Value).TotalSeconds;
        return s < 2 ? "now" : s < 60 ? $"{s:F0} s ago" : s < 3600 ? $"{s / 60:F0} min ago" : s < 86400 ? $"{s / 3600:F1} h ago" : at.Value.ToLocalTime().ToString("yyyy-MM-dd HH:mm");
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

    public static string Location(BinLocation l) => l switch
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
