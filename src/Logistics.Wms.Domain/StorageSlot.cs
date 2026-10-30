namespace Logistics.Wms.Domain;

/// <summary>One bin position in the cylinder shelf.</summary>
public class StorageSlot
{
    public int Id { get; set; }
    public int Column { get; set; }
    public int Level { get; set; }
    /// <summary>Column angle around the R1 base (rad, −π..π, 0 = east port).</summary>
    public double Angle { get; set; }
    /// <summary>Height of the bin floor above the shop floor (m).</summary>
    public double Height { get; set; }
    public string? BinId { get; set; }
    /// <summary>Reserved as destination of an assigned ASRS task.</summary>
    public bool Reserved { get; set; }
    /// <summary>Blocked for maintenance or damage; never used for storage.</summary>
    public bool Blocked { get; set; }

    /// <summary>Human-readable address, column and level counted from 1 (e.g. "C04-L5").</summary>
    public string Code => $"C{Column + 1:00}-L{Level + 1}";

    public bool IsFree => BinId is null && !Reserved && !Blocked;
}
