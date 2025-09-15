
export function binRobotPositions(positions, binSize = 1) {
  const bins = new Map();

  positions.forEach(pos => {
    const xBin = Math.floor(pos.x / binSize);
    const yBin = Math.floor(pos.y / binSize);
    const key = `${xBin},${yBin}`;
    bins.set(key, (bins.get(key) || 0) + 1);
  });

  return Array.from(bins.entries()).map(([key, count]) => {
    const [x, y] = key.split(',').map(Number);
    return {
      x: x * binSize,
      y: y * binSize,
      v: count,
      width: binSize,
      height: binSize,
    };
  });
}