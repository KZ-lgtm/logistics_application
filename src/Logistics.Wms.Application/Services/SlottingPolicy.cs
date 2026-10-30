using Logistics.Wms.Domain;

namespace Logistics.Wms.Application.Services;

/// <summary>
/// Dynamic slotting for R1: a bin goes back into the free slot nearest to the robot's current column (mid levels preferred) instead
/// of a fixed home slot, and a retrieval takes the bin of the SKU that is cheapest to reach, preferring fuller bins.
/// </summary>
public static class SlottingPolicy
{
    /// <summary>Weight of one carton in the bin score: a fuller bin serves more lines per presentation.</summary>
    public const double QuantityWeight = 0.05;

    /// <summary>Swing angle plus weighted height offset from the preferred working height.</summary>
    public static double TravelCost(StorageSlot slot, double robotAngle) =>
        Math.Abs(CellLayout.WrapAngle(slot.Angle - robotAngle)) + CellLayout.HeightWeight * Math.Abs(slot.Height - CellLayout.PreferredHeight);

    public static StorageSlot? ChooseFreeSlot(IEnumerable<StorageSlot> slots, double robotAngle) =>
        slots.Where(s => s.IsFree).OrderBy(s => TravelCost(s, robotAngle)).ThenBy(s => s.Id).FirstOrDefault();

    /// <summary>Best bin among candidates stored in the rack (their <see cref="Bin.Slot"/> must be loaded).</summary>
    public static Bin? ChooseBin(IEnumerable<Bin> candidates, double robotAngle) =>
        candidates
            .Where(b => b.Slot is not null)
            .OrderBy(b => -b.Quantity * QuantityWeight + TravelCost(b.Slot!, robotAngle))
            .ThenBy(b => b.Id, StringComparer.Ordinal)
            .FirstOrDefault();
}
