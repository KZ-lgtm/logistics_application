namespace Logistics.Wms.Domain;

public enum WmsErrorKind { NotFound, Conflict, Validation }

/// <summary>A business-rule violation; the API maps <see cref="Kind"/> to 404 / 409 / 400.</summary>
public sealed class WmsException(WmsErrorKind kind, string message) : Exception(message)
{
    public WmsErrorKind Kind { get; } = kind;

    public static WmsException NotFound(string what, object id) => new(WmsErrorKind.NotFound, $"{what} '{id}' was not found.");
    public static WmsException Conflict(string message) => new(WmsErrorKind.Conflict, message);
    public static WmsException Invalid(string message) => new(WmsErrorKind.Validation, message);
}
