using Logistics.Wms.Application.Services;
using Logistics.Wms.Domain;

namespace Logistics.Wms.Tests;

public class SlottingPolicyTests
{
    private static StorageSlot Slot(int id, int column, int level) =>
        new() { Id = id, Column = column, Level = level, Angle = CellLayout.ColumnAngle(column), Height = CellLayout.LevelHeight(level) };

    [Fact]
    public void Free_slot_is_the_one_nearest_to_the_robot_at_working_height()
    {
        var slots = new[] { Slot(1, 5, 3), Slot(2, 3, 0), Slot(3, 3, 3), Slot(4, 20, 3) };
        Assert.Equal(3, SlottingPolicy.ChooseFreeSlot(slots, CellLayout.ColumnAngle(3))!.Id);
    }

    [Fact]
    public void Occupied_reserved_and_blocked_slots_are_skipped()
    {
        var slots = new[] { Slot(1, 3, 3), Slot(2, 3, 2), Slot(3, 3, 4), Slot(4, 9, 3) };
        slots[0].BinId = "B001";
        slots[1].Reserved = true;
        slots[2].Blocked = true;
        Assert.Equal(4, SlottingPolicy.ChooseFreeSlot(slots, CellLayout.ColumnAngle(3))!.Id);
        Assert.Null(SlottingPolicy.ChooseFreeSlot(slots.Take(3), 0));
    }

    [Fact]
    public void Swing_is_measured_the_short_way_round()
    {
        // column 24 is two columns "behind" column 0 / the east port
        var slots = new[] { Slot(1, 24, 3), Slot(2, 4, 3) };
        Assert.Equal(1, SlottingPolicy.ChooseFreeSlot(slots, 0)!.Id);
    }

    [Fact]
    public void Fuller_bin_wins_when_the_travel_is_similar()
    {
        var a = new Bin { Id = "B001", Sku = null!, Quantity = 4, Slot = Slot(1, 4, 3) };
        var b = new Bin { Id = "B002", Sku = null!, Quantity = 30, Slot = Slot(2, 5, 3) };
        Assert.Equal("B002", SlottingPolicy.ChooseBin([a, b], CellLayout.ColumnAngle(4))!.Id);
    }
}
