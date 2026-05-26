import { useState, useEffect, useRef } from 'react'
import { GRID_W, GRID_H, CELL_SIZE, SHELVES, PRODUCTS, CHARGING_STATION, DISPATCHER, findPath, findPathAlt, isObstacle, findPathWithVisited } from './utils/pathfinding'
/* ───────────────── CONSTANTES DE CODIGO FUENTE ───────────────── */
const highlightCode = (code) => {
  return code
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    // Resaltar palabras clave
    .replace(/\b(function|return|if|else|for|while|let|const|new|continue)\b/g, '<span class="text-blue-700 font-bold">$&</span>')
    // Resaltar nombres de funciones
    .replace(/\b([a-zA-Z_]\w*)(?=\()/g, match => {
      if (['if', 'for', 'while', 'return'].includes(match)) return `<span class="text-blue-700 font-bold">${match}</span>`;
      return `<span class="text-amber-700">${match}</span>`;
    })
    // Resaltar comentarios al final para que no interfieran
    .replace(/(\/\/.*)/g, '<span class="text-emerald-700 italic">$&</span>')
    .replace(/(\/\*[\s\S]*?\*\/)/g, '<span class="text-emerald-700 italic">$&</span>');
};

const TRAD_CODE = `// 1. RECEPCIÓN DE LA ORDEN
function recibirOrdenTradicional(guia) {
  let paquetesPendientes = guia.paquetesDesordenados;
  let destinoFinal = coordenadasDeDespacho();
  
  ejecutarMisionTradicional(paquetesPendientes, destinoFinal);
}

// 2. ANÁLISIS TERRITORIAL (Vecino Más Cercano vía A*)
function encontrarPaqueteMasCercano(posActual, paquetesPendientes) {
  let paqueteMasCercano = null;
  let pasosMinimos = Infinity;
  
  // El radar escanea TODO el almacén evadiendo obstáculos (Ráfaga de CPU)
  // para calcular la distancia de pasos reales hacia cada objetivo restante.
  for (let paquete of paquetesPendientes) {
    let rutaSimulada = calcularRuta_A_Estrella(posActual, paquete.coordenadas);
    
    if (rutaSimulada.pasos < pasosMinimos) {
      pasosMinimos = rutaSimulada.pasos;
      paqueteMasCercano = paquete;
    }
  }
  return paqueteMasCercano;
}

// 3. EJECUCIÓN FÍSICA TRAMO POR TRAMO
function ejecutarMisionTradicional(paquetesPendientes, destinoFinal) {
  let posicionActual = BaseCarga;

  while (paquetesPendientes.length > 0) {
    // A) Análisis territorial masivo para decidir el siguiente destino
    let siguienteObjetivo = encontrarPaqueteMasCercano(posicionActual, paquetesPendientes);
    
    // B) Calcular ruta física final y mover los motores
    let rutaFisica = calcularRuta_A_Estrella(posicionActual, siguienteObjetivo.coordenadas);
    for (let paso of rutaFisica) {
      avanzarMotor(paso);
    }
    
    // C) Recoger paquete y remover de la lista
    activarBrazoMecanico();
    recogerObjeto(siguienteObjetivo);
    paquetesPendientes.eliminar(siguienteObjetivo);
    posicionActual = siguienteObjetivo.coordenadas;
  }
  
  // Finalmente, calcular ruta al despacho
  let rutaAlDespacho = calcularRuta_A_Estrella(posicionActual, destinoFinal);
  for (let paso of rutaAlDespacho) {
    avanzarMotor(paso);
  }
}`;

const MACROP_CODE = `// 1. FASE DE ENTRENAMIENTO PREVIO (Fuera de línea / En el Servidor)
function entrenarMacroOperador(paquetesDesordenados) {
  // En lugar de que el robot piense en vivo, un Servidor Central resuelve el 
  // Problema del Agente Viajero simulando rutas óptimas fuera de línea.
  
  let secuenciaUnida = [];
  let posActual = BaseCarga;
  let pendientes = paquetesDesordenados;
  
  while (pendientes.length > 0) {
    let siguiente = encontrarPaqueteMasCercano(posActual, pendientes);
    let rutaTramo = calcularRuta_A_Estrella(posActual, siguiente.coordenadas);
    
    secuenciaUnida = fusionarRutas(secuenciaUnida, rutaTramo);
    secuenciaUnida.push(ACCION_RECOGER);
    
    pendientes.eliminar(siguiente);
    posActual = siguiente.coordenadas;
  }
  
  let rutaDespacho = calcularRuta_A_Estrella(posActual, Ventanilla);
  secuenciaUnida = fusionarRutas(secuenciaUnida, rutaDespacho);
  
  // Guardamos esta enorme secuencia de acciones como un solo archivo BINARIO ininterrumpido.
  // Es como enseñarle al robot memoria muscular perfecta.
  return new MacroOperador({
    nombre: 'MACROP_RECOLECCION_MASIVA',
    precondiciones: ['EstoyEn(BaseCarga)', 'TodosLosPaquetesDisponibles()'],
    rutinaDirecta: compilarMemoriaMuscular(secuenciaUnida)
  });
}

// 2. RECEPCIÓN DE LA ORDEN
function recibirOrdenMACROP(guia) {
  ejecutarMisionDirecta(guia.paquetesDesordenados);
}

// 3. EJECUCIÓN CONTINUA EN UN SOLO PROCESO
function ejecutarMisionDirecta(paquetesDesordenados) {
  // El MACROP descarga el bloque precompilado desde el servidor 
  let macropEntrenado = entrenarMacroOperador(paquetesDesordenados);
  
  // TOMA DE DECISIÓN INSTANTÁNEA (Uso de Procesador casi a cero)
  if ( validarEstado(macropEntrenado.precondiciones) ) {
    
    // EJECUCIÓN CONTINUA MASIVA: 
    // El robot navega, gira y recolecta todo el inventario de la fábrica
    // como una sola acción refleja, sin analizar el territorio jamás.
    ControladorMotor.ejecutarRutinaDirecta(macropEntrenado.rutinaDirecta);
  }
}`;

/* ───────────────── HELPERS ───────────────── */
const delay = (ms) => new Promise(res => setTimeout(res, ms));

const getCorners = (path) => {
  let corners = [];
  for (let i = 1; i < path.length - 1; i++) {
    const prev = path[i - 1];
    const curr = path[i];
    const next = path[i + 1];
    if (prev.x !== next.x && prev.y !== next.y) {
      corners.push(i);
    }
  }
  return corners;
}

/* ───────────────── COMPONENTES UI ───────────────── */
function TriangularTable({ sequence }) {
  if (!sequence || sequence.length === 0) return null;
  
  const steps = [];
  let currentLoc = 'BaseCarga';
  
  sequence.forEach((p, idx) => {
    steps.push({
      id: `Paso ${idx + 1}`,
      precond: `EstoyEn(${currentLoc})\nPaqueteEnEstante(${p.name})`,
      op: `Mover(${currentLoc} → ${p.name})\nRecoger(${p.name})`,
      effect: `EstoyEn(${p.name})\nTengoPaquete(${p.name})`
    });
    currentLoc = p.name;
  });
  
  steps.push({
    id: `Paso ${sequence.length + 1}`,
    precond: `EstoyEn(${currentLoc})\nTengoPaquetes()`,
    op: `Mover(${currentLoc} → Despacho)\nDescargarTodos()`,
    effect: `EstoyEn(Despacho)\nPaquetesEntregados()`
  });

  return (
    <div className="bg-white rounded-xl shadow-sm border border-slate-200 p-5 mt-6 flex flex-col">
      <div className="flex items-center gap-2 mb-4 pb-3 border-b border-slate-100">
        <svg className="w-5 h-5 text-indigo-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 10h18M3 14h18m-9-4v8m-7 0h14a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
        </svg>
        <h2 className="text-lg font-bold text-slate-800">Tabla Triangular de Dependencias (STRIPS)</h2>
      </div>
      
      <div className="overflow-x-auto">
        <table className="min-w-full text-xs text-left border-collapse border border-slate-200">
          <thead>
            <tr className="bg-slate-100">
              <th className="border border-slate-200 p-3 font-bold w-32 text-slate-700">Estructura Lógica</th>
              {steps.map((s, i) => (
                <th key={i} className="border border-slate-200 p-3 font-bold text-slate-700">{s.id}</th>
              ))}
            </tr>
          </thead>
          <tbody>
            <tr>
              <td className="border border-slate-200 p-3 font-bold bg-slate-50 text-slate-700">Precondiciones</td>
              {steps.map((s, i) => (
                <td key={i} className="border border-slate-200 p-3 whitespace-pre-wrap text-amber-600 font-medium leading-relaxed">{s.precond}</td>
              ))}
            </tr>
            <tr>
              <td className="border border-slate-200 p-3 font-bold bg-slate-50 text-slate-700">Operador (Acción)</td>
              {steps.map((s, i) => (
                <td key={i} className="border border-slate-200 p-3 whitespace-pre-wrap text-indigo-600 font-bold leading-relaxed bg-indigo-50/30">{s.op}</td>
              ))}
            </tr>
            <tr>
              <td className="border border-slate-200 p-3 font-bold bg-slate-50 text-slate-700">Efectos</td>
              {steps.map((s, i) => (
                <td key={i} className="border border-slate-200 p-3 whitespace-pre-wrap text-emerald-600 font-medium leading-relaxed">{s.effect}</td>
              ))}
            </tr>
          </tbody>
        </table>
      </div>

    </div>
  );
}

function CpuGauge({ value, variant }) {
  const isTrad = variant === 'trad';
  const barColor = isTrad 
    ? (value > 70 ? 'bg-red-500' : 'bg-amber-500') 
    : 'bg-emerald-500';
  
  const textColor = isTrad
    ? (value > 70 ? 'text-red-600' : 'text-amber-600')
    : 'text-emerald-600';

  return (
    <div className="space-y-1.5 mb-4 sticky top-0 bg-white pt-2 pb-2 z-10 border-b border-slate-100">
      <div className="flex items-center justify-between">
        <span className="text-[10px] font-bold text-slate-500 uppercase tracking-widest flex items-center gap-1.5">
          <svg className="w-3.5 h-3.5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
          </svg>
          Carga de CPU
        </span>
        <span className={`text-sm font-bold tabular-nums ${textColor}`}>{value}%</span>
      </div>
      <div className="h-2 bg-slate-100 rounded-full overflow-hidden border border-slate-200/60 shadow-inner">
        <div
          className={`h-full ${barColor} transition-all duration-300 ease-out`}
          style={{ width: `${value}%` }}
        />
      </div>
    </div>
  )
}

function LogPanel({ logs, isRunning, variant }) {
  const typeStyles = {
    info: 'text-slate-600 mt-1',
    detail: variant === 'trad' ? 'text-amber-700/80 font-medium ml-1' : 'text-slate-500 italic ml-1',
    warning: variant === 'trad' ? 'text-amber-700 font-bold mt-1' : 'text-slate-600 mt-1',
    success: 'text-emerald-600 font-bold mt-1',
    macro: 'text-indigo-700 font-bold mt-1',
    announcement: 'text-blue-800 font-bold bg-blue-50 p-1.5 rounded border border-blue-100 mb-2 mt-2',
  }

  return (
    <div className="bg-slate-50/50 border border-slate-200 p-4 font-mono text-[12px] leading-relaxed shadow-inner rounded-lg h-auto min-h-[300px] transition-all duration-300">
      {logs.length === 0 && (
        <div className="flex flex-col items-center justify-center h-[200px] text-slate-400 gap-2">
          <svg className="w-6 h-6 opacity-50" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
          </svg>
          <span>Esperando generación de orden...</span>
        </div>
      )}
      <div className="space-y-1.5 flex flex-col justify-end">
        {logs.map((log, i) => (
          <div key={i} className={`flex items-start log-entry-animate ${typeStyles[log.type] || 'text-slate-600'}`}>
            <span className="text-slate-400 mr-2 select-none shrink-0">[{String(i + 1).padStart(2, '0')}]</span>
            <span className="flex-1">{log.text}</span>
            {i === logs.length - 1 && isRunning && <span className="cursor-blink ml-1 text-slate-400" />}
          </div>
        ))}
      </div>
    </div>
  )
}

/* ───────────────── APP PRINCIPAL ───────────────── */

export default function App() {
  const [isRunning, setIsRunning] = useState(false);
  const [targetProducts, setTargetProducts] = useState([])
  const [orderCount, setOrderCount] = useState(0)

  // Trad
  const [tradLogs, setTradLogs] = useState([])
  const [tradCpu, setTradCpu] = useState(0)
  const [tradPos, setTradPos] = useState({ x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY })
  const [tradPath, setTradPath] = useState([])
  const [tradPkgs, setTradPkgs] = useState(0)
  const [tradPkgState, setTradPkgState] = useState('shelf')
  const [tradEvalNode, setTradEvalNode] = useState(null) // Para el ping visual en coordenadas
  
  // Macro
  const [macroLogs, setMacroLogs] = useState([])
  const [macroCpu, setMacroCpu] = useState(0)
  const [macroPos, setMacroPos] = useState({ x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY })
  const [macroPath, setMacroPath] = useState([])
  const [macroPkgs, setMacroPkgs] = useState(0)
  const [macroPkgState, setMacroPkgState] = useState('shelf')
  const [macroEvalNode, setMacroEvalNode] = useState(null)

  const [macropCompiledData, setMacropCompiledData] = useState("")
  const [showMacropData, setShowMacropData] = useState(false)

  const runningRef = useRef(false)

  const addTradLog = (text, type = 'info') => setTradLogs(prev => [...prev, { text, type }])
  const addMacroLog = (text, type = 'info') => setMacroLogs(prev => [...prev, { text, type }])

  const handleGenerateOrder = () => {
    if (isRunning) return;
    
    const numPackages = Math.floor(Math.random() * 3) + 3; // 3 a 5
    let selected = [];
    let available = [...PRODUCTS];
    for (let i = 0; i < numPackages; i++) {
       const idx = Math.floor(Math.random() * available.length);
       selected.push(available[idx]);
       available.splice(idx, 1);
    }
    
    // Optimización de Ruta: Análisis Territorial con A* (Vecino Más Cercano Real)
    let sortedSelected = [];
    let currPos = { x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY };
    let unvisited = [...selected];
    
    while(unvisited.length > 0) {
        let nearestIdx = 0;
        let minSteps = Infinity;
        for(let i=0; i<unvisited.length; i++) {
            // Utilizamos el pathfinding real para saber los "pasos" verdaderos evadiendo obstáculos
            let path = findPath(currPos.x, currPos.y, unvisited[i].x, unvisited[i].y);
            let steps = path.length;
            if(steps > 0 && steps < minSteps) {
                minSteps = steps;
                nearestIdx = i;
            }
        }
        sortedSelected.push(unvisited[nearestIdx]);
        currPos = { x: unvisited[nearestIdx].x, y: unvisited[nearestIdx].y };
        unvisited.splice(nearestIdx, 1);
    }

    const newCount = orderCount + 1;
    setOrderCount(newCount);
    setTargetProducts(sortedSelected);
    
    setTradCpu(0); setMacroCpu(0);
    setTradPos({ x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY });
    setMacroPos({ x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY });
    setTradPath([]); setMacroPath([]);
    setTradPkgs(0); setMacroPkgs(0);
    setTradPkgState('shelf'); setMacroPkgState('shelf');
    setTradEvalNode(null);

    const announcement = `📋 GUÍA #${String(newCount).padStart(4, '0')} | Recolección múltiple (${numPackages} paquetes)`;
    setTradLogs([{ text: announcement, type: 'announcement' }]);
    setMacroLogs([{ text: announcement, type: 'announcement' }]);

    // Generar datos completos de entrenamiento del MACROP
    // Generamos TODAS las permutaciones posibles del orden de recogida
    function permutaciones(arr) {
      if (arr.length <= 1) return [arr];
      let resultado = [];
      for (let i = 0; i < arr.length; i++) {
        let resto = [...arr.slice(0, i), ...arr.slice(i + 1)];
        for (let perm of permutaciones(resto)) {
          resultado.push([arr[i], ...perm]);
        }
      }
      return resultado;
    }

    const todasPermutaciones = permutaciones(sortedSelected);
    const ladoNombre = { Norte: 'Norte', Sur: 'Sur', Este: 'Este', Oeste: 'Oeste' };

    let tLogs = "// ═══════════════════════════════════════════════════════════\n";
    tLogs +=    "// DATOS DE ENTRENAMIENTO - MACRO-OPERADOR\n";
    tLogs +=    "// Resolución del Problema del Agente Viajero (Fuerza Bruta)\n";
    tLogs +=    "// ═══════════════════════════════════════════════════════════\n\n";

    tLogs += `// Dimensiones del almacén: ${GRID_W} x ${GRID_H} celdas\n`;
    tLogs += `// Estanterías registradas: ${SHELVES.length}\n`;
    tLogs += `// Punto de carga: (${CHARGING_STATION.entryX}, ${CHARGING_STATION.entryY})\n`;
    tLogs += `// Ventanilla de despacho: (${DISPATCHER.entryX}, ${DISPATCHER.entryY})\n\n`;

    tLogs += `// --- Paquetes asignados a esta guía ---\n`;
    for (let i = 0; i < sortedSelected.length; i++) {
      const p = sortedSelected[i];
      tLogs += `//   P${i+1}: "${p.name}" | Estante ${p.shelfId}, Lado ${ladoNombre[p.side] || p.side} | Pos: (${p.x}, ${p.y})\n`;
    }

    tLogs += `\n// --- FASE 1: Evaluación exhaustiva de permutaciones ---\n`;
    tLogs += `// Cantidad de órdenes posibles: ${todasPermutaciones.length}\n\n`;

    let mejorCosto = Infinity;
    let mejorIdx = 0;
    let resultadosPermutaciones = [];

    for (let pi = 0; pi < todasPermutaciones.length; pi++) {
      const perm = todasPermutaciones[pi];
      let costoTotal = 0;
      let giros = 0;
      let pos = { x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY };
      let detallesTramos = [];

      for (let j = 0; j < perm.length; j++) {
        const destino = perm[j];
        const tramo = findPath(pos.x, pos.y, destino.x, destino.y);
        const costoTramo = tramo.length;
        const esquinas = getCorners(tramo);
        giros += esquinas.length;
        detallesTramos.push({ nombre: destino.name, estante: destino.shelfId, lado: ladoNombre[destino.side] || destino.side, pasos: costoTramo, girosTramo: esquinas.length });
        costoTotal += costoTramo;
        pos = { x: destino.x, y: destino.y };
      }

      const tramoDespacho = findPath(pos.x, pos.y, DISPATCHER.entryX, DISPATCHER.entryY);
      giros += getCorners(tramoDespacho).length;
      detallesTramos.push({ nombre: 'DESPACHO', estante: '-', lado: '-', pasos: tramoDespacho.length, girosTramo: getCorners(tramoDespacho).length });
      costoTotal += tramoDespacho.length;

      const tramoRetorno = findPath(DISPATCHER.entryX, DISPATCHER.entryY, CHARGING_STATION.entryX, CHARGING_STATION.entryY);
      giros += getCorners(tramoRetorno).length;
      detallesTramos.push({ nombre: 'BASE CARGA', estante: '-', lado: '-', pasos: tramoRetorno.length, girosTramo: getCorners(tramoRetorno).length });
      costoTotal += tramoRetorno.length;

      resultadosPermutaciones.push({ perm, costoTotal, giros, detallesTramos });

      if (costoTotal < mejorCosto) {
        mejorCosto = costoTotal;
        mejorIdx = pi;
      }
    }

    // Ordenar de peor a mejor para mostrar la progresión
    const ordenados = resultadosPermutaciones
      .map((r, i) => ({ ...r, indice: i }))
      .sort((a, b) => b.costoTotal - a.costoTotal);

    for (let oi = 0; oi < ordenados.length; oi++) {
      const r = ordenados[oi];
      const esMejor = r.indice === mejorIdx;
      const orden = r.perm.map((p) => `P${sortedSelected.indexOf(p)+1}`).join(' → ');
      const estado = esMejor ? '✓ ÓPTIMO' : 'Descartada';

      tLogs += `Permutación ${oi + 1}/${todasPermutaciones.length}: [${orden} → Despacho → Base]\n`;
      tLogs += `  ┌─────────────────────────────────────────────────────────\n`;
      for (let d of r.detallesTramos) {
        tLogs += `  │  → ${d.nombre.padEnd(16)} | ${String(d.pasos).padStart(3)} pasos | ${d.girosTramo} giros\n`;
      }
      tLogs += `  └─ Costo total: ${r.costoTotal} pasos | ${r.giros} giros | ${estado}\n\n`;
    }

    // Resultado final
    const mejor = resultadosPermutaciones[mejorIdx];
    tLogs += `\n// --- FASE 2: Ruta óptima seleccionada ---\n`;
    tLogs += `// Orden: ${mejor.perm.map((p) => `P${sortedSelected.indexOf(p)+1}(${p.name})`).join(' → ')}\n`;
    tLogs += `// Costo total: ${mejor.costoTotal} pasos | Giros: ${mejor.giros}\n\n`;

    tLogs += `// --- FASE 3: Compilación de vectores de movimiento ---\n`;

    let posCompile = { x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY };
    let instruccionesTotales = 0;
    for (let j = 0; j < mejor.perm.length; j++) {
      const dest = mejor.perm[j];
      const ruta = findPath(posCompile.x, posCompile.y, dest.x, dest.y);
      tLogs += `\nTramo ${j+1}: (${posCompile.x},${posCompile.y}) → ${dest.name}(${dest.x},${dest.y})\n`;
      tLogs += `Movimientos: `;
      let movs = [];
      for (let k = 1; k < ruta.length; k++) {
        const dx = ruta[k].x - ruta[k-1].x;
        const dy = ruta[k].y - ruta[k-1].y;
        if (dx === 1) movs.push('→');
        else if (dx === -1) movs.push('←');
        else if (dy === 1) movs.push('↓');
        else if (dy === -1) movs.push('↑');
      }
      // Comprimir movimientos repetidos
      let comprimidos = [];
      let i = 0;
      while (i < movs.length) {
        let dir = movs[i];
        let count = 1;
        while (i + count < movs.length && movs[i + count] === dir) count++;
        comprimidos.push(`${dir}x${count}`);
        i += count;
      }
      tLogs += comprimidos.join(', ') + `\n`;
      tLogs += `Acción: recogerPaquete("${dest.name}")\n`;
      instruccionesTotales += ruta.length;
      posCompile = { x: dest.x, y: dest.y };
    }

    const rutaDesp = findPath(posCompile.x, posCompile.y, DISPATCHER.entryX, DISPATCHER.entryY);
    tLogs += `\nTramo final: (${posCompile.x},${posCompile.y}) → Despacho(${DISPATCHER.entryX},${DISPATCHER.entryY})\n`;
    let movsD = [];
    for (let k = 1; k < rutaDesp.length; k++) {
      const dx = rutaDesp[k].x - rutaDesp[k-1].x;
      const dy = rutaDesp[k].y - rutaDesp[k-1].y;
      if (dx === 1) movsD.push('→');
      else if (dx === -1) movsD.push('←');
      else if (dy === 1) movsD.push('↓');
      else if (dy === -1) movsD.push('↑');
    }
    let compD = [];
    let ii = 0;
    while (ii < movsD.length) {
      let dir = movsD[ii]; let count = 1;
      while (ii + count < movsD.length && movsD[ii + count] === dir) count++;
      compD.push(`${dir}x${count}`);
      ii += count;
    }
    tLogs += `Movimientos: ${compD.join(', ')}\n`;
    tLogs += `Acción: descargarTodosLosPaquetes()\n`;
    instruccionesTotales += rutaDesp.length;

    tLogs += `\n// --- FASE 4: Resultado ---\n`;
    tLogs += `// Bloque compilado: MACROP_RECOLECCION.bin\n`;
    tLogs += `// Instrucciones de motor: ${instruccionesTotales}\n`;
    tLogs += `// Permutaciones evaluadas: ${todasPermutaciones.length}\n`;
    tLogs += `// Permutaciones descartadas: ${todasPermutaciones.length - 1}\n`;
    tLogs += `// Uso de procesador del robot en ejecución: 0%\n`;
    tLogs += `// Estado: Listo para ejecución directa\n`;
    
    setMacropCompiledData(tLogs);
    setShowMacropData(false);
  }

  const runSimulation = async () => {
    if (isRunning || targetProducts.length === 0) return;
    setIsRunning(true);
    runningRef.current = true;
    
    setTradCpu(0); setMacroCpu(0);
    setTradPos({ x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY });
    setMacroPos({ x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY });
    setTradPath([]); setMacroPath([]);
    setTradPkgs(0); setMacroPkgs(0);
    setTradPkgState('shelf'); setMacroPkgState('shelf');
    setTradEvalNode(null);

    const announcement = `📋 GUÍA #${String(orderCount).padStart(4, '0')} | Recolección de ${targetProducts.length} paquetes`;
    setTradLogs([{ text: announcement, type: 'announcement' }]);
    setMacroLogs([{ text: announcement, type: 'announcement' }]);

    const MOVEMENT_SPEED_MS = 250; 
    const CORNER_PAUSE_MS = 1000;
    
    // Generar segmentos de ruta
    const waypoints = [
       { x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY },
       ...targetProducts,
       { x: DISPATCHER.entryX, y: DISPATCHER.entryY },
       { x: CHARGING_STATION.entryX, y: CHARGING_STATION.entryY }
    ];

    let segments = [];
    for(let i = 0; i < waypoints.length - 1; i++){
       let p = findPath(waypoints[i].x, waypoints[i].y, waypoints[i+1].x, waypoints[i+1].y);
       segments.push({ path: p, corners: getCorners(p), start: waypoints[i], end: waypoints[i+1] });
    }

    // PROCESO TRADICIONAL
    const runTradSequence = async () => {
      await delay(800);
      
      let currentTradPath = [];
      for(let s = 0; s < segments.length; s++) {
         const seg = segments[s];
         if (!runningRef.current) return;
         
         if (s < targetProducts.length) {
            addTradLog(`🧠 Buscando ruta hacia P${s+1} ('${targetProducts[s].name}')...`, 'warning');
         } else if (s === targetProducts.length) {
            addTradLog(`🧠 Buscando ruta hacia Despacho...`, 'warning');
         } else {
            addTradLog(`🔋 Retornando a Base (Ruta conocida)...`, 'info');
         }
         
         if (s <= targetProducts.length) {
             setTradCpu(100);
             addTradLog(`   ├── Analizando topología y obstáculos (Análisis Territorial)...`, 'detail');
             
             // Escaneo exhaustivo del mapa ignorando estantes (radar completo)
             let currentWave = [];
             // El radar se expande hasta chocar exactamente con la ubicación del próximo paquete o destino
             let targetDist = Math.abs(seg.end.x - seg.start.x) + Math.abs(seg.end.y - seg.start.y);
             for (let radius = 1; radius <= targetDist; radius += 2) {
                currentWave = [];
                for (let x = 0; x < GRID_W; x++) {
                   for (let y = 0; y < GRID_H; y++) {
                      let dist = Math.abs(x - seg.start.x) + Math.abs(y - seg.start.y);
                      if (dist <= radius) {
                         currentWave.push({x, y});
                      }
                   }
                }
                setTradEvalNode({ type: 'flood', nodes: [...currentWave] });
                await delay(70);
             }
             
             await delay(200);
             setTradEvalNode(null);
             addTradLog(`✅ Ruta óptima confirmada. Avanzando.`, 'success');
             setTradCpu(15);
             await delay(400);
         } else {
             setTradCpu(15);
             await delay(300);
         }
         
         currentTradPath = [...currentTradPath, ...seg.path];
         setTradPath(currentTradPath);
         
         const startIdx = (s === 0) ? 0 : 1;
         for (let i = startIdx; i < seg.path.length; i++) {
            if (!runningRef.current) return;
            setTradPos(seg.path[i]);
            
            if (seg.corners.includes(i) && i > 0 && i < seg.path.length - 1) {
               const prev = seg.path[i-1];
               const current = seg.path[i];
               const next = seg.path[i+1];
               
               let straightNode = { x: current.x + (current.x - prev.x), y: current.y + (current.y - prev.y) };
               
               setTradEvalNode({ type: 'corner', redNode: straightNode, greenNode: null });
               addTradLog(`📍 Cruce detectado. Analizando...`, 'detail');
               setTradCpu(85);
               await delay(400); 
               
               setTradEvalNode({ type: 'corner', redNode: straightNode, greenNode: next });
               addTradLog(`   └── Giro confirmado.`, 'warning');
               await delay(600); // 400 + 600 = 1000ms pause
               
               setTradCpu(15);
               setTradEvalNode(null);
            } else {
               await delay(MOVEMENT_SPEED_MS);
            }
         }
         
         if (s < targetProducts.length) {
             addTradLog(`📦 P${s+1} ('${targetProducts[s].name}') recogido.`, 'detail');
             setTradPkgs(s+1);
             setTradPkgState('robot');
             await delay(600);
         } else if (s === targetProducts.length) {
             addTradLog(`✅ Todos los paquetes liberados en despacho.`, 'success');
             setTradPkgs(0);
             setTradPkgState('disp');
             await delay(600);
         } else {
             setTradPath([]);
             setTradCpu(0);
             addTradLog(`💤 Misión finalizada.`, 'info');
         }
      }
    }

    // PROCESO MACROP
    const runMacroSequence = async () => {
      await delay(400);
      addMacroLog(`⚡ Invocando MACROP Múltiple [${targetProducts.length} paquetes]`, 'macro');
      setMacroCpu(45);
      await delay(300);
      addMacroLog(`🧠 Evadiendo cálculo repetitivo. Fusionando ${segments.length} trayectos en 1 bloque de memoria...`, 'detail');
      await delay(500);
      addMacroLog(`✅ Ruta unificada inyectada.`, 'success');
      setMacroCpu(5);
      
      let fullMacroPath = [];
      segments.forEach(s => fullMacroPath.push(...s.path));
      setMacroPath(fullMacroPath);
      await delay(300);
      setMacroCpu(0);
      addMacroLog(`🚀 Ejecutando operador continuo...`, 'info');
      
      for(let s = 0; s < segments.length; s++) {
         const seg = segments[s];
         const startIdx = (s === 0) ? 0 : 1;
         
         for (let i = startIdx; i < seg.path.length; i++) {
            if (!runningRef.current) return;
            setMacroPos(seg.path[i]);
            
            if (seg.corners.includes(i) && i > 0 && i < seg.path.length - 1) {
               setMacroEvalNode(seg.path[i]);
               addMacroLog(`📍 Girando chasis físicamente...`, 'detail');
               await delay(CORNER_PAUSE_MS); // Exactly 1000ms pause
               setMacroEvalNode(null);
            } else {
               await delay(MOVEMENT_SPEED_MS);
            }
         }
         
         if (s < targetProducts.length) {
             addMacroLog(`📦 P${s+1} ('${targetProducts[s].name}') recolectado.`, 'detail');
             setMacroPkgs(s+1);
             setMacroPkgState('robot');
             await delay(600);
         } else if (s === targetProducts.length) {
             addMacroLog(`✅ Paquetes liberados.`, 'success');
             setMacroPkgs(0);
             setMacroPkgState('disp');
             await delay(600);
         } else {
             setMacroPath([]);
             addMacroLog(`💤 Misión MACROP ejecutada. En espera.`, 'info');
         }
      }
    }

    await Promise.all([runTradSequence(), runMacroSequence()]);
    setIsRunning(false);
    runningRef.current = false;
  }

  useEffect(() => { return () => { runningRef.current = false; }; }, []);

  // DRAWING UTILS
  const drawChessboard = () => {
    let board = [];
    for(let y=0; y<GRID_H; y++) {
      for(let x=0; x<GRID_W; x++) {
        const isEven = (x + y) % 2 === 0;
        const color = isEven ? "#ffffff" : "#f1f5f9";
        board.push(<rect key={`bg-${x}-${y}`} x={x * CELL_SIZE} y={y * CELL_SIZE} width={CELL_SIZE} height={CELL_SIZE} fill={color} />);
      }
    }
    return board;
  }

  const drawShelves = () => {
    return SHELVES.map(s => {
      return (
        <g key={s.id}>
          <rect x={s.x * CELL_SIZE} y={s.y * CELL_SIZE} width={s.w * CELL_SIZE} height={s.h * CELL_SIZE} rx="3" fill="#334155" stroke="#1e293b" strokeWidth="1" />
          <text x={(s.x + s.w/2) * CELL_SIZE} y={(s.y + s.h/2) * CELL_SIZE + 4} fill="#cbd5e1" fontSize="12" textAnchor="middle" fontWeight="600">{s.id}</text>
        </g>
      );
    });
  }
  
  const drawRulers = () => {
    let rulers = [];
    const mapH = GRID_H * CELL_SIZE;
    const mapW = GRID_W * CELL_SIZE;
    
    // Eje X (Top & Bottom) cada 1 celda
    for(let x=0; x<GRID_W; x++) {
      // Top
      rulers.push(
        <g key={`rxt-${x}`}>
          <text x={x * CELL_SIZE + CELL_SIZE/2} y={-8} fill="#64748b" fontSize="8" textAnchor="middle" fontWeight="bold">{x}</text>
          <line x1={x * CELL_SIZE + CELL_SIZE/2} y1={-4} x2={x * CELL_SIZE + CELL_SIZE/2} y2={0} stroke="#cbd5e1" strokeWidth="0.5" />
        </g>
      );
      // Bottom
      rulers.push(
        <g key={`rxb-${x}`}>
          <text x={x * CELL_SIZE + CELL_SIZE/2} y={mapH + 12} fill="#64748b" fontSize="8" textAnchor="middle" fontWeight="bold">{x}</text>
          <line x1={x * CELL_SIZE + CELL_SIZE/2} y1={mapH} x2={x * CELL_SIZE + CELL_SIZE/2} y2={mapH + 4} stroke="#cbd5e1" strokeWidth="0.5" />
        </g>
      );
    }
    // Eje Y (Left & Right) cada 1 celda, invertido para que 0 esté abajo
    for(let displayY=0; displayY<GRID_H; displayY++) {
      const actualY = (GRID_H - 1) - displayY;
      // Left
      rulers.push(
        <g key={`ryl-${displayY}`}>
          <text x={-6} y={actualY * CELL_SIZE + CELL_SIZE/2 + 3} fill="#64748b" fontSize="8" textAnchor="end" fontWeight="bold">{displayY}</text>
          <line x1={-4} y1={actualY * CELL_SIZE + CELL_SIZE/2} x2={0} y2={actualY * CELL_SIZE + CELL_SIZE/2} stroke="#cbd5e1" strokeWidth="0.5" />
        </g>
      );
      // Right
      rulers.push(
        <g key={`ryr-${displayY}`}>
          <text x={mapW + 6} y={actualY * CELL_SIZE + CELL_SIZE/2 + 3} fill="#64748b" fontSize="8" textAnchor="start" fontWeight="bold">{displayY}</text>
          <line x1={mapW} y1={actualY * CELL_SIZE + CELL_SIZE/2} x2={mapW + 4} y2={actualY * CELL_SIZE + CELL_SIZE/2} stroke="#cbd5e1" strokeWidth="0.5" />
        </g>
      );
    }
    return rulers;
  }

  const drawZone = (offsetX, variant, pos, path, pkgState, isTrad) => {
    const robotColor = isTrad ? "#f59e0b" : "#10b981";
    const pathColor = isTrad ? "rgba(245, 158, 11, 0.7)" : "rgba(16, 185, 129, 0.7)";
    const transitionMs = 200; 
    const pkgsCollected = isTrad ? tradPkgs : macroPkgs;

    return (
      <g transform={`translate(${offsetX}, 0)`}>
        {drawRulers()}
        {drawChessboard()}
        
        {path.length > 0 && <path d={path.map((n, i) => `${i === 0 ? 'M' : 'L'}${n.x * CELL_SIZE + CELL_SIZE/2},${n.y * CELL_SIZE + CELL_SIZE/2}`).join(' ')} fill="none" stroke={pathColor} strokeWidth="2.5" strokeLinecap="round" strokeLinejoin="round" />}

        {/* Bases (1x1) */}
        <rect x={CHARGING_STATION.x * CELL_SIZE} y={CHARGING_STATION.y * CELL_SIZE} width={CHARGING_STATION.w * CELL_SIZE} height={CHARGING_STATION.h * CELL_SIZE} fill="#eff6ff" stroke="#3b82f6" strokeDasharray="2 2" rx="2" strokeWidth="1"/>
        <text x={(CHARGING_STATION.x + CHARGING_STATION.w/2) * CELL_SIZE} y={(CHARGING_STATION.y + CHARGING_STATION.h/2) * CELL_SIZE + 4} fill="#2563eb" fontSize="10" textAnchor="middle">🔋</text>
        
        <rect x={DISPATCHER.x * CELL_SIZE} y={DISPATCHER.y * CELL_SIZE} width={DISPATCHER.w * CELL_SIZE} height={DISPATCHER.h * CELL_SIZE} fill="#ecfdf5" stroke="#10b981" strokeDasharray="2 2" rx="2" strokeWidth="1"/>
        <text x={(DISPATCHER.x + DISPATCHER.w/2) * CELL_SIZE} y={(DISPATCHER.y + DISPATCHER.h/2) * CELL_SIZE + 4} fill="#059669" fontSize="10" textAnchor="middle">📦</text>

        {drawShelves()}

        {targetProducts.length > 0 && pkgState !== 'disp' && targetProducts.map((prod, idx) => {
           if (idx >= pkgsCollected) {
              return (
                <g key={`prod-${idx}`} transform={`translate(${prod.drawX * CELL_SIZE}, ${prod.drawY * CELL_SIZE})`}>
                  <circle cx="0" cy="0" r="10" fill="#6366f1" opacity="0.3" className="signal-ping" />
                  <rect x="-6" y="-6" width="12" height="12" fill="#6366f1" rx="2" />
                  <text x="0" y="3.5" fill="#fff" fontSize="9" textAnchor="middle" fontWeight="bold">P{idx+1}</text>
                </g>
              );
           }
           return null;
        })}

        {pkgState === 'disp' && (
           <g transform={`translate(${DISPATCHER.entryX * CELL_SIZE + CELL_SIZE/2}, ${DISPATCHER.entryY * CELL_SIZE + CELL_SIZE/2})`}>
             <rect x="-6" y="-6" width="16" height="16" fill="#10b981" rx="2" />
             <text x="0" y="4" fill="#fff" fontSize="10" textAnchor="middle" fontWeight="bold">{targetProducts.length}</text>
           </g>
        )}

        {/* Ping Visual de Evaluación de Coordenadas (Solo en Zona Tradicional) */}
        {isTrad && tradEvalNode && tradEvalNode.type === 'corner' && (
          <g>
            {tradEvalNode.redNode && (
              <g transform={`translate(${tradEvalNode.redNode.x * CELL_SIZE + CELL_SIZE/2}, ${tradEvalNode.redNode.y * CELL_SIZE + CELL_SIZE/2})`}>
                 <circle cx="0" cy="0" r="10" fill="none" stroke="#ef4444" strokeWidth="2.5" className="animate-ping" />
                 <rect x="-4" y="-4" width="8" height="8" fill="#ef4444" opacity="0.9" />
              </g>
            )}
            {tradEvalNode.greenNode && (
              <g transform={`translate(${tradEvalNode.greenNode.x * CELL_SIZE + CELL_SIZE/2}, ${tradEvalNode.greenNode.y * CELL_SIZE + CELL_SIZE/2})`}>
                 <circle cx="0" cy="0" r="10" fill="none" stroke="#22c55e" strokeWidth="2.5" className="animate-ping" />
                 <rect x="-4" y="-4" width="8" height="8" fill="#22c55e" opacity="0.9" />
              </g>
            )}
          </g>
        )}
        
        {isTrad && tradEvalNode && tradEvalNode.type === 'flood' && tradEvalNode.nodes.map((n, idx) => (
          <g key={`flood-${idx}`} transform={`translate(${n.x * CELL_SIZE + CELL_SIZE/2}, ${n.y * CELL_SIZE + CELL_SIZE/2})`}>
             <circle cx="0" cy="0" r="8" fill="#f59e0b" opacity="0.5" />
          </g>
        ))}

        {/* Ping Visual de Giro (Solo MACROP) */}
        {!isTrad && macroEvalNode && (
          <g transform={`translate(${macroEvalNode.x * CELL_SIZE + CELL_SIZE/2}, ${macroEvalNode.y * CELL_SIZE + CELL_SIZE/2})`}>
             <circle cx="0" cy="0" r="10" fill="none" stroke="#22c55e" strokeWidth="2.5" className="animate-ping" />
             <rect x="-4" y="-4" width="8" height="8" fill="#22c55e" opacity="0.9" />
          </g>
        )}

        {/* Robot */}
        <g transform={`translate(${pos.x * CELL_SIZE}, ${pos.y * CELL_SIZE})`} style={{ transition: `transform ${transitionMs}ms linear` }}>
          <rect x={2} y={2} width={CELL_SIZE - 4} height={CELL_SIZE - 4} fill={robotColor} rx="3" />
          {/* Render packages on robot */}
          {pkgState === 'robot' && Array.from({length: pkgsCollected}).map((_, idx) => (
             <g key={`carry-${idx}`} transform={`translate(${CELL_SIZE/2}, ${CELL_SIZE/2 - 4 - (idx * 3)})`}>
               <rect x="-5" y="-5" width="10" height="10" fill="#6366f1" rx="1.5" />
             </g>
          ))}
        </g>
      </g>
    )
  }

  const mapWidth = GRID_W * CELL_SIZE; 
  const mapHeight = GRID_H * CELL_SIZE;
  // Aumentamos el tamaño total para incluir márgenes de las reglas (rulers) y un gap central.
  const gap = 80;
  const totalSvgWidth = (mapWidth * 2) + gap;

  return (
    <div className="min-h-screen bg-slate-100 font-sans text-slate-800 p-6">
      <div className="max-w-[1500px] mx-auto space-y-6">
        
        {/* ENUNCIADO Y CONTROLES PRINCIPALES */}
        <div className="bg-white rounded-xl shadow-sm border border-slate-200 p-5 flex flex-col sm:flex-row items-center justify-between gap-6">
          <div className="flex-1 flex flex-col justify-center">
            <h1 className="text-xl font-bold text-slate-800 flex items-center gap-2 mb-3">
              <svg className="w-5 h-5 text-indigo-500" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 11V9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10" />
              </svg>
              Emisión de Guías y Monitoreo Logístico
            </h1>
            
            {targetProducts.length > 0 ? (
              <div className="bg-slate-50 border border-slate-200 rounded-lg p-4 inline-block">
                <p className="text-sm font-semibold text-slate-500 uppercase tracking-wider mb-2">Orden en curso: Guía #{String(orderCount).padStart(4, '0')}</p>
                <p className="text-lg text-slate-800">
                  Operación de recolección de <strong className="text-indigo-700">{targetProducts.length} paquetes</strong>:
                </p>
                <div className="flex flex-wrap gap-2 mt-3">
                  {targetProducts.map((p, idx) => {
                    const ladoStr = { top: 'Arriba', bottom: 'Abajo', left: 'Izquierda', right: 'Derecha' }[p.side] || p.side;
                    return (
                      <span key={idx} className="bg-indigo-100 text-indigo-800 px-2 py-1 rounded text-sm font-medium border border-indigo-200">
                        P{idx+1}: {p.name} (Estante {p.shelfId} - Lado: {ladoStr})
                      </span>
                    );
                  })}
                </div>
              </div>
            ) : (
              <p className="text-base text-slate-500 italic">El sistema está inactivo. Presione generar orden para comenzar la demostración.</p>
            )}
          </div>
          
          <div className="flex flex-col sm:flex-row gap-3">
            <button
              onClick={handleGenerateOrder}
              disabled={isRunning}
              className="px-6 py-3 bg-indigo-600 text-white rounded-lg font-bold hover:bg-indigo-700 active:scale-95 transition-all shadow-sm flex items-center justify-center gap-2 disabled:opacity-60 disabled:cursor-not-allowed text-lg"
            >
              📝 Generar Nueva Guía
            </button>
          </div>
        </div>

        {/* CENTRO DE LOGISTICA */}
        <div className="bg-white rounded-xl shadow-sm border border-slate-200 p-5 flex flex-col">
          <div className="flex flex-col sm:flex-row justify-between items-start sm:items-center mb-5 pb-3 border-b border-slate-100 gap-4">
            <h2 className="text-xl font-bold text-slate-800 flex items-center gap-2">
              <svg className="w-6 h-6 text-slate-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7" />
              </svg>
              Centro de Logística
            </h2>
            {targetProducts.length > 0 && (
              <button
                onClick={runSimulation}
                disabled={isRunning}
                className="px-6 py-2 bg-emerald-50 border border-emerald-200 text-emerald-700 rounded-lg font-bold hover:bg-emerald-100 active:scale-95 transition-all flex items-center gap-2 disabled:opacity-50 text-base"
              >
                ▶ Ejecutar
              </button>
            )}
          </div>
          
          <div className="w-full flex justify-center bg-slate-50 p-4 rounded-lg border border-slate-100 inset-shadow-sm flex-1 overflow-hidden">
             <svg viewBox={`-25 -65 ${totalSvgWidth + 50} ${mapHeight + 100}`} className="w-full h-auto bg-white shadow-sm rounded-md border border-slate-200 pt-4 pl-4 pr-4 pb-4">
                
                {/* Labels Zonas */}
                <rect x={mapWidth/2 - 120} y={-52} width="240" height="30" fill="white" rx="6" opacity="0.95" stroke="#e2e8f0" />
                <text x={mapWidth / 2} y={-32} fill="#64748b" fontSize="14" fontWeight="bold" textAnchor="middle" letterSpacing="1">ZONA TRADICIONAL</text>
                
                <rect x={mapWidth + gap + mapWidth/2 - 120} y={-52} width="240" height="30" fill="white" rx="6" opacity="0.95" stroke="#e2e8f0" />
                <text x={mapWidth + gap + mapWidth / 2} y={-32} fill="#64748b" fontSize="14" fontWeight="bold" textAnchor="middle" letterSpacing="1">ZONA MACROP</text>

                {/* Zona 1: Offset = 0 */}
                {drawZone(0, 'trad', tradPos, tradPath, tradPkgState, true)}
                
                {/* Línea Divisoria */}
                <line x1={mapWidth + gap/2} y1="-25" x2={mapWidth + gap/2} y2={mapHeight + 25} stroke="#cbd5e1" strokeWidth="2" strokeDasharray="6 6" />

                {/* Zona 2: Offset = mapWidth + gap */}
                {drawZone(mapWidth + gap, 'macro', macroPos, macroPath, macroPkgState, false)}
             </svg>
          </div>
        </div>

        {/* TABLAS DE PROCESOS */}
        <div className="bg-white rounded-xl shadow-sm border border-slate-200 p-5 flex flex-col">
          <div className="flex flex-col sm:flex-row justify-between items-start sm:items-center mb-5 pb-3 border-b border-slate-100 gap-4">
            <h2 className="text-xl font-bold text-slate-800 flex items-center gap-2">
              <svg className="w-6 h-6 text-slate-400" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 20l4-16m4 4l4 4-4 4M6 16l-4-4 4-4" />
              </svg>
              Procesamientos
            </h2>
            {targetProducts.length > 0 && (
              <button
                onClick={runSimulation}
                disabled={isRunning}
                className="px-6 py-2 bg-emerald-50 border border-emerald-200 text-emerald-700 rounded-lg font-bold hover:bg-emerald-100 active:scale-95 transition-all flex items-center gap-2 disabled:opacity-50 text-base"
              >
                ▶ Ejecutar
              </button>
            )}
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 items-start">
             <div className="flex flex-col">
               <div className="flex items-center gap-2 mb-2">
                 <div className="w-2.5 h-2.5 rounded-full bg-orange-500" />
                 <h3 className="font-bold text-slate-800 text-sm">Registro Tradicional</h3>
               </div>
               <div className="relative">
                 <CpuGauge value={tradCpu} variant="trad" />
                 <LogPanel logs={tradLogs} isRunning={isRunning} variant="trad" />
               </div>
             </div>
             
             <div className="flex flex-col">
               <div className="flex items-center gap-2 mb-2">
                 <div className="w-2.5 h-2.5 rounded-full bg-emerald-500" />
                 <h3 className="font-bold text-slate-800 text-sm">Registro MACROP</h3>
               </div>
               <div className="relative">
                 <CpuGauge value={macroCpu} variant="macro" />
                 <LogPanel logs={macroLogs} isRunning={isRunning} variant="macro" />
               </div>
             </div>
          </div>
        </div>

        {/* TABLA TRIANGULAR */}
        <TriangularTable sequence={targetProducts} />

        {/* ALGORITMOS / CÓDIGO FUENTE (VS CODE LIGHT THEME) */}
        <div className="bg-white rounded-xl shadow-md border border-slate-200 p-6 flex flex-col text-slate-800 mt-6">
          <div className="flex items-center gap-3 mb-6 pb-4 border-b border-slate-100">
            <svg className="w-6 h-6 text-indigo-600" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 20l4-16m4 4l4 4-4 4M6 16l-4-4 4-4" />
            </svg>
            <h2 className="text-xl font-bold text-slate-800 tracking-wide">Código</h2>
          </div>

          <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
            {/* Tradicional Code */}
            <div className="flex flex-col gap-3">
              <div className="flex items-center gap-2">
                <div className="w-3 h-3 rounded-full bg-orange-500"></div>
                <h3 className="font-semibold text-slate-700">Algoritmo Tradicional</h3>
              </div>
              <div className="bg-[#f8f9fa] p-5 rounded-md font-mono text-xs overflow-x-auto border border-slate-200 shadow-inner">
                <pre className="text-slate-800 leading-relaxed" dangerouslySetInnerHTML={{ __html: highlightCode(TRAD_CODE) }}></pre>
              </div>
            </div>

            {/* MACROP Code */}
            <div className="flex flex-col gap-3">
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-emerald-500"></div>
                  <h3 className="font-semibold text-slate-700">Algoritmo MACROP</h3>
                </div>
                {targetProducts.length > 0 && (
                  <button 
                    onClick={() => setShowMacropData(!showMacropData)}
                    className="text-xs bg-emerald-100 hover:bg-emerald-200 text-emerald-800 px-3 py-1 rounded font-bold transition-colors"
                  >
                    {showMacropData ? "Ver Código" : "Ver Consola de Entrenamiento"}
                  </button>
                )}
              </div>
              <div className="bg-[#f8f9fa] p-5 rounded-md font-mono text-xs overflow-x-auto border border-slate-200 shadow-inner min-h-[300px]">
                {showMacropData ? (
                  <pre className="text-slate-800 leading-relaxed whitespace-pre-wrap">{macropCompiledData}</pre>
                ) : (
                  <pre className="text-slate-800 leading-relaxed" dangerouslySetInnerHTML={{ __html: highlightCode(MACROP_CODE) }}></pre>
                )}
              </div>
            </div>
          </div>
        </div>

      </div>
    </div>
  )
}
