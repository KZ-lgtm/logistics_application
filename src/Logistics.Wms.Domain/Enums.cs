namespace Logistics.Wms.Domain;

/// <summary>Carton size class; decides how many cartons fit in one storage bin.</summary>
public enum SkuSize { S, M, L }

/// <summary>Where a storage bin physically is.</summary>
public enum BinLocation
{
    /// <summary>Stored in a slot of the cylinder shelf.</summary>
    Rack,
    /// <summary>On the picking loop (east port), heading for a picking stop.</summary>
    PickInbound,
    /// <summary>Standing at picking stop P1 or P2 under the 3D camera.</summary>
    AtStation,
    /// <summary>Back at the picking-loop return head, waiting for R1 to store it.</summary>
    PickReturn,
    /// <summary>At the restock operator outside the west port.</summary>
    RestockStation,
    /// <summary>Refilled, back at the restock-loop return head, waiting for R1 to store it.</summary>
    RestockReturn,
}

public enum PickStation { P1, P2 }

public enum Sorter { S1, S2 }

public enum OrderStatus { Queued, Active, Completed, Cancelled }

/// <summary>State of an order case on a put-wall lane.</summary>
public enum LaneState
{
    /// <summary>Empty case in position, no order yet.</summary>
    Ready,
    /// <summary>Case in position and collecting items for an order.</summary>
    Active,
    /// <summary>Order finished, case running out to the packing operator.</summary>
    Outbound,
    /// <summary>Finished case at the packing operator.</summary>
    AtOperator,
    /// <summary>Empty case running in.</summary>
    Inbound,
}

public enum AsrsTaskKind
{
    /// <summary>Rack slot → picking loop.</summary>
    Retrieve,
    /// <summary>Picking-loop return head → rack slot.</summary>
    Store,
    /// <summary>Empty bin from the picking-loop return head → restock loop.</summary>
    ReturnToRestock,
    /// <summary>Empty bin from a rack slot → restock loop.</summary>
    RackToRestock,
    /// <summary>Refilled bin from the restock-loop return head → rack slot.</summary>
    StoreRestocked,
}

public enum AsrsTaskStatus { Assigned, Completed, Cancelled }

/// <summary>Life cycle of one picked carton (one piece of one order line).</summary>
public enum PickStatus
{
    /// <summary>Allocated in the bin at the picking stop, not yet picked.</summary>
    Pending,
    /// <summary>Picked onto the item belt, not yet scanned.</summary>
    OnBelt,
    /// <summary>Bar code read at the arch, routed to a sorter.</summary>
    Routed,
    /// <summary>Put into the order case.</summary>
    Sorted,
    /// <summary>Ran off the belt into the exception tote.</summary>
    Exception,
    /// <summary>Allocation released before the pick (order cancelled).</summary>
    Cancelled,
}

public enum ExceptionReason { NoRead, BarcodeMismatch, OrderNotActive, MissedSorter }

public enum ExceptionDisposition { Open, ReturnedToStock, Scrapped }

public enum InventoryTransactionType
{
    InitialStock,
    /// <summary>Carton picked out of a bin (−1).</summary>
    Pick,
    /// <summary>Cartons put into a bin at the restock station (from the stock pallet and the exception buffer).</summary>
    Restock,
    /// <summary>Cycle count or count at a picking stop.</summary>
    Adjustment,
    /// <summary>Carton from the exception tote returned to stock (into the SKU's restock buffer, +1).</summary>
    ReturnFromException,
    /// <summary>Sum of older bookings of a bin (or a SKU's restock buffer) that were folded together by history compaction.</summary>
    OpeningBalance,
}
