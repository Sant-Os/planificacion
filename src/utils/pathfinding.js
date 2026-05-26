// src/utils/pathfinding.js

export const GRID_W = 33;
export const GRID_H = 34;
export const CELL_SIZE = 16;

export const SHELVES = [];
let shelfId = 1;

// 5 x 5 = 25 estantes
const cols = [3, 9, 15, 21, 27];
const rows = [3, 9, 15, 21, 27];

for (let r=0; r<rows.length; r++) {
  for (let c=0; c<cols.length; c++) {
    // 3 de ancho x 4 de alto
    const w = 3;
    const h = 4;
    const x = cols[c];
    const y = rows[r];

    SHELVES.push({ id: `E${shelfId++}`, x, y, w, h });
  }
}

const SIDES = ['Norte', 'Sur', 'Este', 'Oeste'];
export const PRODUCTS = [];
let prodId = 1;
for (let shelf of SHELVES) {
  const side = SIDES[Math.floor(Math.random() * SIDES.length)];
  let targetX, targetY;
  let drawX, drawY;

  if (side === 'Norte') { 
      targetX = shelf.x + 1; 
      targetY = shelf.y - 1; 
      drawX = shelf.x + 1.5;
      drawY = shelf.y;
  } else if (side === 'Sur') { 
      targetX = shelf.x + 1; 
      targetY = shelf.y + shelf.h; 
      drawX = shelf.x + 1.5;
      drawY = shelf.y + shelf.h;
  } else if (side === 'Este') { 
      targetX = shelf.x + shelf.w; 
      targetY = shelf.y + 1; 
      drawX = shelf.x + shelf.w;
      drawY = shelf.y + 1.5;
  } else { 
      targetX = shelf.x - 1; 
      targetY = shelf.y + 1; 
      drawX = shelf.x;
      drawY = shelf.y + 1.5;
  }

  PRODUCTS.push({
    id: `P${prodId++}`,
    shelfId: shelf.id,
    name: `Paquete_${prodId}`,
    x: targetX,
    y: targetY,
    drawX,
    drawY,
    side: side
  });
}

// Carga inferior izquierda (1x1)
export const CHARGING_STATION = { x: 0, y: 33, w: 1, h: 1, name: 'CARGA', entryX: 1, entryY: 33 };
// Ventanilla superior derecha (1x1)
export const DISPATCHER = { x: 32, y: 0, w: 1, h: 1, name: 'VENTANILLA', entryX: 31, entryY: 0 };

export function isObstacle(x, y) {
  if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return true;
  
  for (const shelf of SHELVES) {
    if (x >= shelf.x && x < shelf.x + shelf.w && y >= shelf.y && y < shelf.y + shelf.h) {
      return true;
    }
  }

  if (x >= CHARGING_STATION.x && x < CHARGING_STATION.x + CHARGING_STATION.w && y >= CHARGING_STATION.y && y < CHARGING_STATION.y + CHARGING_STATION.h) {
      return true;
  }
  
  if (x >= DISPATCHER.x && x < DISPATCHER.x + DISPATCHER.w && y >= DISPATCHER.y && y < DISPATCHER.y + DISPATCHER.h) {
      return true;
  }

  return false;
}

export function findPath(startX, startY, targetX, targetY) {
  class Node {
    constructor(x, y, parent = null) {
      this.x = x;
      this.y = y;
      this.parent = parent;
      this.g = 0;
      this.h = 0;
      this.f = 0;
    }
  }

  const startNode = new Node(startX, startY);
  const endNode = new Node(targetX, targetY);

  let openList = [startNode];
  let closedList = new Set();

  const getNeighbors = (node) => {
    // Orden de evaluación: Derecha, Arriba, Izquierda, Abajo
    const dirs = [[1, 0], [0, -1], [-1, 0], [0, 1]]; 
    let result = [];
    for (let d of dirs) {
      const nx = node.x + d[0];
      const ny = node.y + d[1];
      if (!isObstacle(nx, ny) || (nx === targetX && ny === targetY)) {
        result.push(new Node(nx, ny, node));
      }
    }
    return result;
  };

  while (openList.length > 0) {
    let currentIndex = 0;
    for (let i = 1; i < openList.length; i++) {
      if (openList[i].f < openList[currentIndex].f) {
        currentIndex = i;
      }
    }

    let currentNode = openList[currentIndex];

    if (currentNode.x === endNode.x && currentNode.y === endNode.y) {
      let path = [];
      let current = currentNode;
      while (current != null) {
        path.push({ x: current.x, y: current.y });
        current = current.parent;
      }
      return path.reverse();
    }

    openList.splice(currentIndex, 1);
    closedList.add(`${currentNode.x},${currentNode.y}`);

    let neighbors = getNeighbors(currentNode);
    for (let neighbor of neighbors) {
      if (closedList.has(`${neighbor.x},${neighbor.y}`)) {
        continue;
      }

      let costIncrement = 1;
      // Penalización por giro: Si la dirección de movimiento cambia, añadir un costo extra
      if (currentNode.parent) {
        const dx1 = currentNode.x - currentNode.parent.x;
        const dy1 = currentNode.y - currentNode.parent.y;
        const dx2 = neighbor.x - currentNode.x;
        const dy2 = neighbor.y - currentNode.y;
        
        if (dx1 !== dx2 || dy1 !== dy2) {
          costIncrement += 2; // Penalizar fuertemente los giros
        }
      }

      neighbor.g = currentNode.g + costIncrement;
      neighbor.h = Math.abs(neighbor.x - endNode.x) + Math.abs(neighbor.y - endNode.y);
      neighbor.f = neighbor.g + neighbor.h;

      let inOpenWithLowerG = false;
      for (let openNode of openList) {
        if (openNode.x === neighbor.x && openNode.y === neighbor.y && openNode.g <= neighbor.g) {
          inOpenWithLowerG = true;
          break;
        }
      }

      if (!inOpenWithLowerG) {
        openList.push(neighbor);
      }
    }
  }

  return []; 
}

export function findPathAlt(startX, startY, targetX, targetY, avoidPath = []) {
  class Node {
    constructor(x, y, parent = null) {
      this.x = x;
      this.y = y;
      this.parent = parent;
      this.g = 0;
      this.h = 0;
      this.f = 0;
    }
  }

  const startNode = new Node(startX, startY);
  const endNode = new Node(targetX, targetY);

  let openList = [startNode];
  let closedList = new Set();
  
  const avoidSet = new Set(avoidPath.map(n => `${n.x},${n.y}`));

  const getNeighbors = (node) => {
    // Orden invertido para variar un poco los empates
    const dirs = [[0, -1], [1, 0], [-1, 0], [0, 1]]; 
    let result = [];
    for (let d of dirs) {
      const nx = node.x + d[0];
      const ny = node.y + d[1];
      if (!isObstacle(nx, ny) || (nx === targetX && ny === targetY)) {
        result.push(new Node(nx, ny, node));
      }
    }
    return result;
  };

  while (openList.length > 0) {
    let currentIndex = 0;
    for (let i = 1; i < openList.length; i++) {
      if (openList[i].f < openList[currentIndex].f) {
        currentIndex = i;
      }
    }

    let currentNode = openList[currentIndex];

    if (currentNode.x === endNode.x && currentNode.y === endNode.y) {
      let path = [];
      let current = currentNode;
      while (current != null) {
        path.push({ x: current.x, y: current.y });
        current = current.parent;
      }
      return path.reverse();
    }

    openList.splice(currentIndex, 1);
    closedList.add(`${currentNode.x},${currentNode.y}`);

    let neighbors = getNeighbors(currentNode);
    for (let neighbor of neighbors) {
      if (closedList.has(`${neighbor.x},${neighbor.y}`)) {
        continue;
      }

      let costIncrement = 1;
      
      // Si el nodo pertenece a la ruta original, lo penalizamos masivamente
      // para forzar al algoritmo a buscar por otro lado.
      if (avoidSet.has(`${neighbor.x},${neighbor.y}`)) {
        costIncrement += 50; 
      }

      if (currentNode.parent) {
        const dx1 = currentNode.x - currentNode.parent.x;
        const dy1 = currentNode.y - currentNode.parent.y;
        const dx2 = neighbor.x - currentNode.x;
        const dy2 = neighbor.y - currentNode.y;
        if (dx1 !== dx2 || dy1 !== dy2) costIncrement += 2; 
      }

      neighbor.g = currentNode.g + costIncrement;
      neighbor.h = Math.abs(neighbor.x - endNode.x) + Math.abs(neighbor.y - endNode.y);
      neighbor.f = neighbor.g + neighbor.h;

      let inOpenWithLowerG = false;
      for (let openNode of openList) {
        if (openNode.x === neighbor.x && openNode.y === neighbor.y && openNode.g <= neighbor.g) {
          inOpenWithLowerG = true;
          break;
        }
      }

      if (!inOpenWithLowerG) openList.push(neighbor);
    }
  }

  return []; 
}

export function findPathWithVisited(startX, startY, targetX, targetY) {
  class Node {
    constructor(x, y, parent = null) {
      this.x = x;
      this.y = y;
      this.parent = parent;
      this.g = 0;
      this.h = 0;
      this.f = 0;
    }
  }

  const startNode = new Node(startX, startY);
  const endNode = new Node(targetX, targetY);

  let openList = [startNode];
  let closedList = new Set();
  let visitedNodes = []; // Para la animación

  const getNeighbors = (node) => {
    const dirs = [[1, 0], [0, -1], [-1, 0], [0, 1]];
    let result = [];
    for (let d of dirs) {
      const nx = node.x + d[0];
      const ny = node.y + d[1];
      if (!isObstacle(nx, ny) || (nx === targetX && ny === targetY)) {
        result.push(new Node(nx, ny, node));
      }
    }
    return result;
  };

  while (openList.length > 0) {
    let currentIndex = 0;
    for (let i = 1; i < openList.length; i++) {
      if (openList[i].f < openList[currentIndex].f) {
        currentIndex = i;
      }
    }

    let currentNode = openList[currentIndex];

    if (currentNode.x === endNode.x && currentNode.y === endNode.y) {
      let path = [];
      let current = currentNode;
      while (current != null) {
        path.push({ x: current.x, y: current.y });
        current = current.parent;
      }
      return { path: path.reverse(), visitedNodes };
    }

    openList.splice(currentIndex, 1);
    closedList.add(`${currentNode.x},${currentNode.y}`);
    visitedNodes.push({ x: currentNode.x, y: currentNode.y });

    let neighbors = getNeighbors(currentNode);
    for (let neighbor of neighbors) {
      if (closedList.has(`${neighbor.x},${neighbor.y}`)) {
        continue;
      }

      let costIncrement = 1;
      if (currentNode.parent) {
        const dx1 = currentNode.x - currentNode.parent.x;
        const dy1 = currentNode.y - currentNode.parent.y;
        const dx2 = neighbor.x - currentNode.x;
        const dy2 = neighbor.y - currentNode.y;
        if (dx1 !== dx2 || dy1 !== dy2) {
          costIncrement += 2;
        }
      }

      neighbor.g = currentNode.g + costIncrement;
      neighbor.h = Math.abs(neighbor.x - endNode.x) + Math.abs(neighbor.y - endNode.y);
      neighbor.f = neighbor.g + neighbor.h;

      let inOpenWithLowerG = false;
      for (let openNode of openList) {
        if (openNode.x === neighbor.x && openNode.y === neighbor.y && openNode.g <= neighbor.g) {
          inOpenWithLowerG = true;
          break;
        }
      }

      if (!inOpenWithLowerG) openList.push(neighbor);
    }
  }

  return { path: [], visitedNodes };
}
