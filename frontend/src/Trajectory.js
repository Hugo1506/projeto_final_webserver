import React, { useRef, useEffect } from 'react';

function concentrationToColor(concentration, min = 0, max = 1) {
  const norm = Math.max(0, Math.min(1, (concentration - min) / (max - min)));
  let hue;
  if (norm <= 0.25) {
    hue = 240 - norm * 120; 
  } else if (norm <= 0.5) {
    hue = 120 - (norm - 0.25) * 240;
  } else if (norm <= 0.75) {
    hue = 60 - (norm - 0.5) * 60; 
  } else {
    hue = 0; 
  }

  return `hsl(${hue}, 100%, 50%)`;
}

const Trajectory = ({
  robotPaths,
  simulationBounds,
  width,
  height,
  currentIteration,
}) => {
  const canvasRef = useRef(null);

  const visiblePoints = robotPaths.flatMap(path =>
    path.filter(p => p.iteration <= currentIteration)
  );

  const concentrations = visiblePoints.map(p => Number(p.concentration));
  const minConc = Math.min(...concentrations, 0);
  const maxConc = Math.max(...concentrations, 1);

  function mapToCanvas(x, y) {
    const { xMin, xMax, yMin, yMax } = simulationBounds;
    const xRange = xMax - xMin;
    const yRange = yMax - yMin;

    const px = ((y - yMin) / yRange) * width;  
    const py = ((x - xMin) / xRange) * height; 

    return [px, py];
  }

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, width, height);

    ctx.strokeStyle = '#ccc';
    ctx.lineWidth = 2;
    ctx.strokeRect(0, 0, width, height);

    robotPaths.forEach((path, robotIndex) => {
      const visiblePath = path.filter(p => p.iteration <= currentIteration);
      if (visiblePath.length < 2) return;

      ctx.lineWidth = 3;

      for (let i = 1; i < visiblePath.length; i++) {
        const prev = visiblePath[i - 1];
        const curr = visiblePath[i];
        const [x1, y1] = mapToCanvas(prev.robot_position.x, prev.robot_position.y);
        const [x2, y2] = mapToCanvas(curr.robot_position.x, curr.robot_position.y);

        ctx.strokeStyle = concentrationToColor(curr.concentration, minConc, maxConc);

        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(x2, y2, 6, 0, 2 * Math.PI);
        ctx.fillStyle = ctx.strokeStyle;
        ctx.fill();
      }

      const lastPoint = visiblePath[visiblePath.length - 1];
      const [lastX, lastY] = mapToCanvas(lastPoint.robot_position.x, lastPoint.robot_position.y);
      ctx.font = 'bold 12px Arial';
      ctx.fillStyle = '#000';
      ctx.fillText(`R${lastPoint.robot ?? robotIndex}`, lastX + 8, lastY + 4); 
    });
  }, [robotPaths, simulationBounds, width, height, minConc, maxConc, currentIteration]);

  return (
    <div style={{ background: '#fff', padding: '10px', borderRadius: '8px', boxShadow: '0 0 8px #eee' }}>
      <canvas
        ref={canvasRef}
        width={width}
        height={height}
        style={{ background: '#fff', border: '1px solid #ccc', display: 'block', margin: '0 auto' }}
      />
      <div style={{ textAlign: 'center', marginTop: '8px', fontSize: '13px', color: '#555' }}>
        Robot path colored by concentration
      </div>
    </div>
  );
};

export default Trajectory;
