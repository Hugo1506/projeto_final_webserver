import React, { useEffect, useRef, useState } from 'react';

const EnviromentGrid = ({ gifObj, simulationBounds, grid, height, onSetPlumeCoords }) => {
  const canvasRef = useRef(null);
  const [clickCoords, setClickCoords] = useState(null); 

  useEffect(() => {
    if (!simulationBounds) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    ctx.clearRect(0, 0, width, height); // Clear previous drawings

    const { xMin, xMax, yMin, yMax } = simulationBounds;
    const xRange = xMax - xMin;
    const yRange = yMax - yMin;

    if (grid) {
      // Draw horizontal lines for x (height)
      for (let x = xMin; x <= xMax; x += 0.1) {
        const py = ((x - xMin) / xRange) * height;
        ctx.beginPath();
        ctx.moveTo(0, py);
        ctx.lineTo(width, py);
        ctx.strokeStyle = 'rgba(72, 255, 0, 1)';
        ctx.lineWidth = 1;
        ctx.stroke();
      }

      // Draw vertical lines for y (width)
      for (let y = yMin; y <= yMax; y += 0.1) {
        const px = ((y - yMin) / yRange) * width;
        ctx.beginPath();
        ctx.moveTo(px, 0);
        ctx.lineTo(px, height);
        ctx.strokeStyle = 'rgba(72, 255, 0, 1)';
        ctx.lineWidth = 1;
        ctx.stroke();
      }
    }

    // If there's a clicked point, draw a red marker
    if (clickCoords) {
      const { x, y } = clickCoords;
      const markerX = ((y - yMin) / yRange) * width;
      const markerY = ((x - xMin) / xRange) * height;

      // Draw red marker at the clicked location
      ctx.beginPath();
      ctx.arc(markerX, markerY, 8, 0, 2 * Math.PI);
      ctx.fillStyle = 'red';
      ctx.fill();
      ctx.strokeStyle = 'black';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [simulationBounds, gifObj.url, grid, clickCoords]);

  const handleCanvasClick = (e) => {
    if (!simulationBounds || !onSetPlumeCoords) return;

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const { xMin, xMax, yMin, yMax } = simulationBounds;
    const xRange = xMax - xMin; // height range
    const yRange = yMax - yMin; // width range

    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;  // horizontal (width)
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height; // vertical (height)

    const simX = xMin + (py / canvas.height) * xRange;
    const simY = yMin + (px / canvas.width) * yRange;

    // Snap to lower left of the 0.1x0.1 grid square
    const snappedX = Math.floor((simX - xMin) / 0.1) * 0.1 + xMin;
    const snappedY = Math.floor((simY - yMin) / 0.1) * 0.1 + yMin;

    // Set the clicked coordinates and trigger the callback
    setClickCoords({ x: snappedX, y: snappedY });
    const plumeZ = parseFloat(height); // assuming height here is the z-coordinate
    onSetPlumeCoords(snappedX, snappedY, plumeZ);
  };

  return (
    <div style={{ position: 'relative', display: 'inline-block' }}>
      <img
        src={gifObj.url}
        alt="Simulation GIF"
        className="gif-image"
        style={{ display: 'block' }}
      />
      <canvas
        ref={canvasRef}
        width={400}
        height={400}
        style={{
          position: 'absolute',
          top: 0,
          left: 0,
          pointerEvents: 'auto',
          width: '100%',
          height: '100%',
        }}
        onClick={handleCanvasClick}
      />
    </div>
  );
};

export default EnviromentGrid;
