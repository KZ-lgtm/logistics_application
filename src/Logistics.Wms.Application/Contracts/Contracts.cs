using Logistics.Wms.Domain;

namespace Logistics.Wms.Application.Contracts;

// ---------------------------------------------------------------- orders
public sealed record CreateOrderLineRequest(string SkuCode, int Quantity);

public sealed record CreateOrderRequest(string Reference, IReadOnlyList<CreateOrderLineRequest> Lines, int Priority = 0);

public sealed record OrderLineDto(int Id, string SkuCode, int Quantity, int Allocated, int Picked, int Sorted);

public sealed record OrderDto(
    int Id, string Reference, int Priority, OrderStatus Status, string? LaneId,
    DateTime CreatedAt, DateTime? ActivatedAt, DateTime? CompletedAt, int TotalQuantity, int SortedQuantity,
    IReadOnlyList<OrderLineDto> Lines);

// ---------------------------------------------------------------- ASRS (R1)
public sealed record AsrsTaskDto(
    int Id, AsrsTaskKind Kind, AsrsTaskStatus Status, string BinId, string SkuCode, int BinQuantity,
    string? FromSlot, string? ToSlot, string From, string To, DateTime CreatedAt, DateTime? CompletedAt);

// ---------------------------------------------------------------- picking (R2 / R3)
public enum RouteDecision { Station, Return, Wait }

public sealed record BinRouteDto(string BinId, RouteDecision Decision, PickStation? Station, int OpenDemand);

public sealed record PickInstructionDto(long PickTaskId, int Sequence, int OrderId, string OrderReference, string SkuCode, PickStatus Status);

public sealed record StationDto(PickStation Station, string? BinId, string? SkuCode, int BinQuantity, IReadOnlyList<PickInstructionDto> Picks);

public sealed record PickConfirmationDto(long PickTaskId, string BinId, int BinQuantityLeft, int PendingInBin, bool BinReleased);

public sealed record BinCountRequest(int CountedQuantity, string? User = null);

// ---------------------------------------------------------------- sortation (bar-code arch, S1 / S2)
public sealed record ScanRequest(long PickTaskId, string? Barcode);

public enum ScanTarget { S1, S2, Exception }

public sealed record ScanResultDto(long PickTaskId, ScanTarget Target, string? LaneId, int? OrderId, ExceptionReason? ExceptionReason);

public sealed record SortConfirmRequest(long PickTaskId, string LaneId);

/// <summary>A carton on the item belt: not yet scanned (<see cref="PickStatus.OnBelt"/>) or routed to its sorter and lane.</summary>
public sealed record BeltItemDto(long PickTaskId, string SkuCode, int OrderId, PickStatus Status, Sorter? TargetSorter, string? TargetLaneId, DateTime? PickedAt);

public sealed record SortConfirmationDto(long PickTaskId, string LaneId, int OrderId, bool OrderCompleted);

// ---------------------------------------------------------------- put wall
public sealed record LaneDto(string Id, Sorter Sorter, int Index, LaneState State, bool Disabled, int? OrderId, string? OrderReference,
    int OrderItems, int SortedItems, int CasesCompleted, DateTime? StateSince);

public enum LaneEvent { CaseAtOperator, CaseReplaced, EmptyCaseInPosition }

// ---------------------------------------------------------------- restock
public sealed record RestockRequest(int? Quantity = null, string? SkuCode = null, string? User = null);

public sealed record RestockBinDto(string BinId, string SkuCode, int Quantity, int Capacity, int SuggestedAdd, int BufferAvailable);

// ---------------------------------------------------------------- inventory
public sealed record SkuStockDto(
    int SkuId, string Code, string Name, SkuSize Size, int BinCapacity, int Bins, int InRack, int OnLoops, int Total,
    int PendingPicks, int OpenDemand, int RestockBuffer, int ReorderPoint, bool LowStock);

public sealed record BinDto(string Id, string SkuCode, int Quantity, int Capacity, BinLocation Location, string? Slot,
    PickStation? Station, bool Reserved, DateTime? LastMovedAt);

public sealed record SlotDto(int Id, string Code, int Column, int Level, double Angle, double Height, string? BinId, bool Reserved, bool Blocked);

public sealed record CreateSkuRequest(string Code, string Name, SkuSize Size, double WeightKg, int ReorderPoint = 0);

public sealed record AdjustBinRequest(int CountedQuantity, string? Reason = null, string? User = null);

public sealed record InventoryTransactionDto(long Id, DateTime At, InventoryTransactionType Type, string SkuCode, string? BinId,
    int Quantity, int? BinQuantityAfter, string? Reference, string? User);

// ---------------------------------------------------------------- exceptions
public sealed record ExceptionItemDto(int Id, long PickTaskId, string SkuCode, int OrderId, ExceptionReason Reason, string? ReadBarcode,
    ExceptionDisposition Disposition, DateTime CreatedAt, DateTime? ResolvedAt, string? ResolvedBy);

public sealed record ResolveExceptionRequest(ExceptionDisposition Disposition, string? User = null);

// ---------------------------------------------------------------- robots
public sealed record RobotDto(
    string Id, string Model, RobotRole Role, string Assignment, string Tool, string Controller,
    RobotMode Mode, RobotState State, bool Available, string? FaultCode, string? FaultText, DateTime? FaultSince,
    string? TaskRef, string? TaskText, DateTime? TaskStartedAt,
    int Cycles, double? AverageCycleSeconds, double LastCycleSeconds, double Utilization, double Availability, int Faults, DateTime StatsSince);

public sealed record RobotEventDto(long Id, string RobotId, DateTime At, RobotEventKind Kind, string? Code, string Text, string? User);

public sealed record SetRobotModeRequest(RobotMode Mode, string? User = null);

public sealed record RobotFaultRequest(string Code, string? Text = null, string? User = null);

/// <summary>A unit of work the WMS hands to one robot: an ASRS move (R1), a pick (R2 / R3) or a put into an order case (S1 / S2).</summary>
public sealed record RobotTaskDto(
    string RobotId, string Kind, string Ref, string Description, string? BinId, string? SkuCode, int? OrderId, string? LaneId, string? From, string? To);

/// <summary>
/// What the cell controller sees at R1's transfer heads when it asks for the next task. With it the WMS only stores a bin that is
/// physically at a return head and only puts a bin on a free out head. Without it (null) every bin booked at a head counts.
/// </summary>
public sealed record AsrsHeads(string? PickReturnBin, string? RestockReturnBin, bool PickOutFree = true, bool RestockOutFree = true);

// ---------------------------------------------------------------- dashboard
public sealed record DashboardDto(
    DateTime At,
    OrderKpis Orders,
    ThroughputKpis Throughput,
    StorageKpis Storage,
    IReadOnlyList<LaneDto> Lanes,
    IReadOnlyList<StationDto> Stations,
    AsrsTaskDto? CurrentAsrsTask,
    ExceptionKpis Exceptions,
    IReadOnlyList<SkuStockDto> LowStock,
    IReadOnlyList<RobotDto> Robots);

public sealed record OrderFigures(int Queued, int Active, int Completed, double? AverageLeadTimeSeconds, int LastHour);

public sealed record OrderKpis(int Queued, int Active, int Completed, int Cancelled, double? AverageLeadTimeSeconds);

public sealed record ThroughputKpis(int OrdersLastHour, int ItemsSortedLastHour, int ItemsPickedTotal, int ItemsSortedTotal,
    int ItemsOnBelt, int Scans, int NoReads, double NoReadRate, double ItemsPerPresentation);

public sealed record StorageKpis(int Slots, int FreeSlots, int BinsInRack, int EmptyBinsInRack, int BinsOnPickLoop, int BinsOnRestockLoop,
    int ItemsInStock);

public sealed record ExceptionKpis(int Open, int ToteCount, int ToteCapacity, bool ToteFull);
