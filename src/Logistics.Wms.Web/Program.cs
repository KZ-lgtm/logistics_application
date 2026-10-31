using Logistics.Wms.Application;
using Logistics.Wms.Application.Abstractions;
using Logistics.Wms.Application.Simulation;
using Logistics.Wms.Persistence;
using Logistics.Wms.Web;
using Logistics.Wms.Web.Services;
using Microsoft.AspNetCore.Components.Web;
using Microsoft.AspNetCore.Components.WebAssembly.Hosting;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Logging;

var builder = WebAssemblyHostBuilder.CreateDefault(args);
builder.RootComponents.Add<App>("#app");
builder.RootComponents.Add<HeadOutlet>("head::after");
builder.Logging.AddFilter("Microsoft.EntityFrameworkCore", LogLevel.Warning);

// the same domain and WMS services as the server edition; the database is EF Core in-memory, saved to local storage
builder.Services.AddWmsApplication(new SimulationOptions { StepMilliseconds = 400, Seed = Environment.TickCount });
builder.Services.AddDbContext<WmsDbContext>(o => o.UseInMemoryDatabase("logistics-wms"));
builder.Services.AddScoped<IWmsDbContext>(sp => sp.GetRequiredService<WmsDbContext>());
var clock = new SimClock();
builder.Services.AddSingleton(clock);
builder.Services.AddSingleton<TimeProvider>(clock);     // replaces TimeProvider.System: WMS time follows the live 3D cell
builder.Services.AddSingleton<Notifications>();
builder.Services.AddSingleton<WmsClient>();
builder.Services.AddSingleton<BrowserStore>();
builder.Services.AddSingleton<SimulationRunner>();
builder.Services.AddSingleton<TwinBridge>();

var host = builder.Build();
await host.Services.GetRequiredService<BrowserStore>().LoadAsync();
_ = host.Services.GetRequiredService<BrowserStore>().RunAutosaveAsync(TimeSpan.FromSeconds(3), CancellationToken.None);
_ = host.Services.GetRequiredService<SimulationRunner>().RunAsync(CancellationToken.None);
await host.RunAsync();
