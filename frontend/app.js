/* ============================================================
    app.js - ThingSpeak -> Chart.js (fields 1..8)
    - STEP charts: field1 (motor), field3 (led) -> no puntos, stepped
    - Line charts: puntos pequeños (pointRadius=3), tooltip muestra "Etiqueta: valor"
    - Bar charts: esquinas redondeadas, tooltip con "Etiqueta: valor"
    - Theme toggle: tema NEON <-> tema BONES (blanco-hueso)
    - Giros Izquierda redondeado a entero
============================================================ */

/* CONFIG */
const CHANNEL_ID = "3163848";
const READ_API_KEY = "1P9PQPO0AAU7B130";
const FETCH_URL = `https://api.thingspeak.com/channels/${CHANNEL_ID}/feeds.json?api_key=${READ_API_KEY}`;

/* UI REFS */
const UI = {
  updateTime: document.getElementById("updateTime"),
  health: document.getElementById("healthValue"),
  motorState: document.getElementById("motorStateValue"),
  ledState: document.getElementById("ledStateValue"),
  rightRot: document.getElementById("rightRotValue"),
  leftRot: document.getElementById("leftRotValue"),
  userReq: document.getElementById("userReqValue"),
  themeToggle: document.getElementById("themeToggle")
};

/* ESTADO */
let system = {
  field1: 0, field2: 0, field3: 0, field4: 0, field5: 0, field6: 0, field7: 0, field8: 0,
  timeLabels: [],
  cache: { f1: [], f2: [], f3: [], f4: [], f5: [], f6: [], f7: []}
};

/* CHARTS */
let charts = {};

/* HELPERS */
function fmtHora(iso) {
  const d = new Date(iso);
  return d.toLocaleTimeString("es-ES", { hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

/* CREATORS */
// STEP chart (no puntos)
function createStepChart(canvasId, label, color) {
  return new Chart(document.getElementById(canvasId), {
    type: "line",
    data: { labels: [], datasets: [{ label: label, data: [], borderColor: color, borderWidth: 2, stepped: true, pointRadius: 0 }] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      scales: {
        x: { ticks: { color: "#9ca3af" } },
        y: { ticks: { color: "#9ca3af", stepSize: 1 }, min: 0, max: 2 }
      },
      plugins: {
        tooltip: {
          callbacks: {
            label: ctx => `${label.replace('Line Chart - ', '')}: ${ctx.formattedValue}`
          }
        },
        legend: { labels: { color: "#e5e5e5" } }
      }
    }
  });
}

// Line chart with small points and tooltip
function createLineChartWithPoints(canvasId, label, color) {
  return new Chart(document.getElementById(canvasId), {
    type: "line",
    data: { labels: [], datasets: [{ label: label, data: [], borderColor: color, backgroundColor: color, borderWidth: 2, tension: 0.25, pointRadius: 3, pointBackgroundColor: color }] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        tooltip: {
          callbacks: {
            label: ctx => `${label.replace('Line Chart - ', '')}: ${ctx.formattedValue}`
          }
        },
        legend: { labels: { color: "#e5e5e5" } }
      },
      scales: {
        x: { ticks: { color: "#9ca3af" } },
        y: { ticks: { color: "#9ca3af" } }
      }
    }
  });
}

// Bar chart with rounded corners
function createBarChart(canvasId, label, color) {
  return new Chart(document.getElementById(canvasId), {
    type: "bar",
    data: { labels: [], datasets: [{ label: label, data: [], backgroundColor: color, borderRadius: 8, borderSkipped: false }] },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      plugins: {
        tooltip: {
          callbacks: {
            label: ctx => `${label.replace('Bar Chart - ', '')}: ${ctx.formattedValue}`
          }
        },
        legend: { labels: { color: "#e5e5e5" } }
      },
      scales: {
        x: { ticks: { color: "#9ca3af" } },
        y: { ticks: { color: "#9ca3af" } }
      }
    }
  });
}

/* CREATE ALL CHARTS */
function createAllCharts() {
  const neon = "#4ADE80";
  charts.f1 = createStepChart("chart_motorState", "Line Chart - Estado Motor", neon);
  charts.f2 = createLineChartWithPoints("chart_motorProgress", "Line Chart - Progreso del Giro Actual", neon);
  charts.f3 = createStepChart("chart_ledState", "Line Chart - Estado LED", neon);
  charts.f4 = createBarChart("chart_rightRot", "Bar Chart - Giros Derecha", neon);
  charts.f5 = createBarChart("chart_leftRot", "Bar Chart - Giros Izquierda", neon);
  charts.f6 = createLineChartWithPoints("chart_health", "Line Chart - Estado del Dispositivo", neon);
  charts.f7 = createBarChart("chart_userReq", "Bar Chart - Peticiones Usuario", neon);
}

/* UPDATE UI (divs) */
function updateUI() {
  UI.health.textContent = system.field6 + "%";
  UI.motorState.textContent = system.field1 === 0 ? "DETENIDO" : system.field1 === 1 ? "GIRO IZQUIERDA" : "GIRO DERECHA";
  UI.ledState.textContent = system.field3 === 1 ? "ENCENDIDO" : "APAGADO";
  UI.rightRot.textContent = system.field4;
  UI.leftRot.textContent = Math.round(system.field5); // redondeo a entero
  UI.userReq.textContent = system.field7;
  UI.updateTime.textContent = new Date().toLocaleTimeString('es-ES');
}

/* UPDATE CHARTS */
function updateCharts() {
  charts.f1.data.labels = system.timeLabels;
  charts.f1.data.datasets[0].data = system.cache.f1;
  charts.f1.update();

  charts.f2.data.labels = system.timeLabels;
  charts.f2.data.datasets[0].data = system.cache.f2;
  charts.f2.options.scales.y.min = 0;
  charts.f2.options.scales.y.max = 100;
  charts.f2.update();

  charts.f3.data.labels = system.timeLabels;
  charts.f3.data.datasets[0].data = system.cache.f3;
  charts.f3.options.scales.y.min = 0;
  charts.f3.options.scales.y.max = 1;
  charts.f3.options.scales.y.ticks.stepSize = 1;
  charts.f3.update();

  charts.f4.data.labels = system.timeLabels;
  charts.f4.data.datasets[0].data = system.cache.f4;
  charts.f4.update();

  charts.f5.data.labels = system.timeLabels;
  charts.f5.data.datasets[0].data = system.cache.f5.map(v => Math.round(v)); // redondeo a entero
  charts.f5.update();

  charts.f6.data.labels = system.timeLabels;
  charts.f6.data.datasets[0].data = system.cache.f6;
  charts.f6.options.scales.y.min = 0;
  charts.f6.options.scales.y.max = 100;
  charts.f6.update();

  charts.f7.data.labels = system.timeLabels;
  charts.f7.data.datasets[0].data = system.cache.f7;
  charts.f7.update();

}

/* FETCH THINGSPEAK */
async function fetchThingSpeak() {
  try {
    const res = await fetch(FETCH_URL, { cache: 'no-store' });
    const json = await res.json();
    if (!json.feeds) return;

    const feeds = json.feeds;
    system.timeLabels = [];
    for (let k in system.cache) system.cache[k] = [];

    feeds.forEach(feed => {
      const label = fmtHora(feed.created_at);
      system.timeLabels.push(label);
      system.cache.f1.push(Number(feed.field1 || 0));
      system.cache.f2.push(Number(feed.field2 || 0));
      system.cache.f3.push(Number(feed.field3 || 0));
      system.cache.f4.push(Number(feed.field4 || 0));
      system.cache.f5.push(Number(feed.field5 || 0)); // redondeo aplicado en updateCharts
      system.cache.f6.push(Number(feed.field6 || 0));
      system.cache.f7.push(Number(feed.field7 || 0));
    });

    const last = feeds[feeds.length - 1];
    system.field1 = Number(last.field1 || 0);
    system.field2 = Number(last.field2 || 0);
    system.field3 = Number(last.field3 || 0);
    system.field4 = Number(last.field4 || 0);
    system.field5 = Number(last.field5 || 0);
    system.field6 = Number(last.field6 || 0);
    system.field7 = Number(last.field7 || 0);

    updateUI();
    updateCharts();
  } catch (err) {
    console.error("Error ThingSpeak:", err);
  }
}

/* THEME TOGGLE */
function toggleTheme() {
  document.body.classList.toggle('bones');
  UI.themeToggle.textContent = document.body.classList.contains('bones') ? 'Volver a Neon' : 'Tema ThingSpeak (hueso)';
  Object.values(charts).forEach(ch => ch.update());
}

/* INIT */
document.addEventListener('DOMContentLoaded', () => {
  createAllCharts();
  fetchThingSpeak();
  setInterval(fetchThingSpeak, 3000);
  setInterval(() => UI.updateTime && (UI.updateTime.textContent = new Date().toLocaleTimeString('es-ES')), 1000);
  UI.themeToggle.addEventListener('click', toggleTheme);
});
