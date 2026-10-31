https://kz-lgtm.github.io/logistics_application/wms/
# Logistics application — Warehouse Management System (C# / .NET 10)

A warehouse management system (WMS) for the goods-to-robot order-fulfilment cell in [`simulation/`](simulation/), the live 3D
simulation and ISO 12100 risk assessment of the cell:

* **R1** IRB 6700 with an L-fork inside a **cylinder shelf** (26 columns × 8 levels, 144 slots; the east port feeds the picking loop, the west port the restock loop)
* **R2 / R3** IRB 1300 with 3D vision and vacuum grippers at picking stops **P1 / P2**
* an item belt with a **bar-code arch** and an **exception tote** at the belt end
* **S1 / S2** IRB 930 SCARAs putting cartons into order cases on an **8-lane put wall**
* a **restock operator** outside the west port and a **packing operator** at the put wall

The WMS owns the stock, the orders and every decision the cell needs. It comes in two editions that share the same domain model
and WMS services:

| | **Server edition** (`Logistics.Wms.Api`) | **Browser edition** (`Logistics.Wms.Web`) |
|---|---|---|
| Runs as | ASP.NET Core service | Blazor WebAssembly app, static files (GitHub Pages) |
| Database | SQLite | EF Core in-memory, saved to the browser's local storage |
| Used by | the real cell: the cell controller (WCS / PLC / operator panels) reports events and asks for the next task over HTTP | people: demonstration, training, process studies, with the built-in simulator or a hand-operated cell console |

```
src/
  Logistics.Wms.Domain          entities, enums, cell layout constants (no dependencies)
  Logistics.Wms.Application     WMS services, contracts, slotting policy, cell simulator
  Logistics.Wms.Persistence     EF Core model (WmsDbContext), seeding, JSON snapshots (provider-neutral)
  Logistics.Wms.Infrastructure  SQLite provider for the server edition
  Logistics.Wms.Api             server edition: ASP.NET Core minimal API, OpenAPI document, live dashboard (wwwroot)
  Logistics.Wms.Web             browser edition: Blazor WebAssembly app for GitHub Pages
tests/
  Logistics.Wms.Tests           xUnit: unit, process-flow, long simulation runs (SQLite and in-memory), snapshots,
                                history compaction, HTTP integration
site/                           GitHub Pages landing page
simulation/                     the original 3D cell simulation and risk assessment (unchanged)
```

## Quick start

Requires the .NET 10 SDK.

### Browser edition

Published at `https://<owner>.github.io/<repo>/wms/` by the Pages workflow (see [GitHub Pages](#github-pages)). Locally:

```bash
dotnet run --project src/Logistics.Wms.Web
```

Open the URL it prints and press **Run cell**, or create orders and work through **Cell operations** by hand. Its pages:

* **Dashboard**: KPIs, put wall, R1 task, stops, storage, exceptions, low stock, and the simulator settings.
* **Orders**: create (several lines, priority), filter by status, line details, cancel.
* **Cell operations**: every cell event as a button, in process order: R1 task done or cancelled, decision point, arrival at a stop, pick confirmation, 3D count correction, scan (OK / no-read / wrong code), sort, missed carton, lane steps, restock (fill level, new SKU), exception disposition.
* **Inventory**: stock per SKU, new SKUs, bins with cycle counts, the stock ledger.
* **Cylinder shelf**: top view of all 144 slots coloured by fill, R1's heading, slot details, block/unblock.
* **Data & storage**: save status, JSON export/import, history compaction, reset.

State lives in the browser's local storage, separately for each browser. History is compacted automatically once there are more than 600 orders.

### Server edition

```bash
dotnet run --project src/Logistics.Wms.Api --urls http://localhost:5213
```

* Dashboard: <http://localhost:5213/>. Press **Start simulation** to let the built-in cell simulator drive the WMS.
* OpenAPI document: <http://localhost:5213/openapi/v1.json>

On first start the SQLite database `wms.db` is created and seeded with the same data as the 3D simulation: 40 SKUs (carton sizes
S / M / L), 144 slots, 8 lanes, and 3 bins per SKU filled to 25–100 %.

```bash
dotnet test
```

## Process and the decisions the WMS makes

| Step | Cell event / request | WMS decision |
|---|---|---|
| Order intake | `POST /api/orders` | Validates SKUs and merges duplicate lines. **Rolling waves**: every lane with an empty case takes the next queued order (highest priority first, then FIFO). |
| Demand | – | Adds up the open lines of all active orders per SKU and subtracts what bins already on their way to a stop will cover. |
| R1 | `POST /api/asrs/tasks/next`, `…/{id}/complete` | One task at a time, in this order: (1) a bin at the picking-loop return head: empty → restock loop, else store it; (2) a refilled bin from restock → store; (3) **retrieve** for uncovered demand while the picking loop has fewer than 7 bins; (4) an empty rack bin → restock (at most 3 on the loop). **Dynamic slotting**: a bin goes to the free slot with the shortest swing from R1's current column, mid levels preferred. A retrieval takes the bin that is cheapest to reach, preferring fuller bins. |
| Decision point | `POST /api/picking/bins/{bin}/route` | P2, unless a bin is already at or heading to P2; then P1; otherwise wait. If the SKU has no open demand any more, the bin goes straight back. |
| Picking stop | `POST /api/picking/stations/{P1\|P2}/arrive/{bin}` | **Allocates** the bin's cartons to the oldest open lines, one pick task per carton. The pick-task id is the carton's tracking number from then on. |
| Pick | `POST /api/picking/picks/{id}/confirm` | Books the stock out of the bin. The last pick releases the bin to the return head. |
| 3D count differs | `POST /api/picking/bins/{bin}/count` | Corrects the stock. Picks the bin can no longer serve go back to demand, and another bin is fetched. |
| Bar-code arch | `POST /api/sortation/scan` | Routes the carton to S1 / S2 and its lane. A no-read, a wrong bar code or an order that is no longer active sends it to the **exception tote**, and its line is reopened. |
| Sort | `POST /api/sortation/sort` | Checks the lane and order. The last carton completes the order and sends the case out. |
| Packing operator | `POST /api/putwall/lanes/{id}/events/{CaseAtOperator\|CaseReplaced\|EmptyCaseInPosition}` | Lane state machine; an empty case in position takes the next order. |
| Restock operator | `GET /api/restock`, `POST /api/restock/{bin}/complete` | Refills the bin (default: full) and uses cartons returned from exceptions first. An empty bin may be given another SKU. |
| Exceptions | `GET /api/exceptions`, `POST /api/exceptions/{id}/resolve`, `POST /api/exceptions/tote/empty` | *Returned to stock* puts the carton in the SKU's restock buffer; *scrapped* removes it. Tote fill level is tracked (capacity 25). |

Also included:

* **Inventory**: stock per SKU with reorder points, bins, slots (block/unblock for maintenance), cycle counts, and a full stock ledger (`/api/inventory/...`).
* **Dashboard**: KPIs such as orders and items per hour, lead time, items per bin presentation and no-read rate (`GET /api/dashboard`).
* **Cancellation**: an order can be cancelled while queued, or while active as long as no carton has been picked. Its allocations go back to the wave and its lane takes the next order.

Errors are RFC 7807 problem details: 404 for unknown resources, 409 for state conflicts (such as a wrong lane, an occupied stop or an out-of-order lane event) and 400 for invalid input.

## Simulator

`CellSimulator` stands in for the cell controller. Each round, every actor takes at most one step through the public services,
using the same order generator as the 3D simulation (Pareto or flat demand, up to 4 lines × 3 pieces, configurable no-read rate). It
drives the dashboard demo (`POST /api/simulation/start|stop|step`, or `"Simulation:Enabled": true`) and the long-run tests.

The long-run tests check these invariants after 1,500 rounds:

* the stock ledger equals bins plus restock buffer for every SKU
* every picked carton is on the belt, in a case or in the exception tote
* slots and bins are consistent with each other
* loop capacities hold, and each stop holds at most one bin
* lanes and active orders match one to one
* R1 never has more than one task

## GitHub Pages

`.github/workflows/pages.yml` builds and publishes the static site:

| Path | Content |
|---|---|
| `/` | landing page (`site/`) |
| `/wms/` | the browser edition (`dotnet publish src/Logistics.Wms.Web`) |
| `/simulation/` | the standalone 3D cell simulation and the risk assessment |

The workflow sets the app's `<base href>` from the Pages base path, so it works under any repository name or a custom domain. GitHub Pages
has no fallback to `index.html`, so the root `404.html` forwards deep links such as `/wms/orders` into the app.

To turn it on, set **Settings → Pages → Build and deployment → Source** to **GitHub Actions**. Then push, or run the workflow from the
Actions tab.

## Configuration (`appsettings.json`)

| Key | Default | |
|---|---|---|
| `ConnectionStrings:Wms` | `Data Source=wms.db` | SQLite database |
| `Simulation:Enabled` | `false` | run the simulator in the background at start-up |
| `Simulation:StepMilliseconds` | `500` | time between simulator rounds |
| `Simulation:ParetoDemand` / `NoReadRate` / `MaxLinesPerOrder` / `MaxQuantityPerLine` / `QueueTarget` / `Seed` | | order generator |

## Design notes and limitations

* **Single command gate.** Every state-changing request (anything except GET) runs under one `WmsCommandGate`, and so does each simulator round. Decisions like "which bin does R1 fetch next" read and then change shared state, and a single cell produces only a few commands per second, so serialising them is the simplest correct choice. Several cells would need one gate per cell.
* **Schema creation.** The schema is created with `EnsureCreated`. Add EF Core migrations before you need to evolve a production database.
* **No authentication** yet. Put the API behind the plant network or add authentication before exposing it.
* Cell geometry (slot angles and heights, gap columns, capacities) lives in `CellLayout` and matches `simulation/canvas/project/app.js`.
* **Browser edition.** It has no server and no shared state: each browser has its own cell, and local storage can be cleared by the user or the browser. Use JSON export for anything worth keeping. The in-memory provider has no indexes, which is why history is compacted early. `HistoryService` keeps lifetime KPIs and folds old ledger entries into opening balances, so stock stays exact. The first visit downloads the .NET runtime (about 10 MB); later visits load from the browser cache.
