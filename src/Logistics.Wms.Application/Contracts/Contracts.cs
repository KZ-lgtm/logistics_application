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
    IReadOnlyList<SkuStockDto> LowStock);

public sealed record OrderKpis(int Queued, int Active, int Completed, int Cancelled, double? AverageLeadTimeSeconds);

public sealed record ThroughputKpis(int OrdersLastHour, int ItemsSortedLastHour, int ItemsPickedTotal, int ItemsSortedTotal,
    int ItemsOnBelt, int Scans, int NoReads, double NoReadRate, double ItemsPerPresentation);

public sealed record StorageKpis(int Slots, int FreeSlots, int BinsInRack, int EmptyBinsInRack, int BinsOnPickLoop, int BinsOnRestockLoop,
    int ItemsInStock);

public sealed record ExceptionKpis(int Open, int ToteCount, int ToteCapacity, bool ToteFull);
