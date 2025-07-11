import React, { useRef, useEffect, useState } from 'react';

const GifWithGrid = ({ gifObj, simulationBounds, robots, onSetRobotCoords }) => {
  const canvasRef = useRef(null);
  const [hoveredRobot, setHoveredRobot] = useState(null);  // Track the hovered robot
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });  // Position of the tooltip

  // Array of predefined colors for robots
  const robotColors = [
    'red', 'blue', 'green', 'orange', 'purple', 'yellow', 'pink', 'brown', 'cyan', 'magenta'
  ];

  useEffect(() => {
    if (!simulationBounds) return;
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    ctx.clearRect(0, 0, width, height);

    const { xMin, xMax, yMin, yMax } = simulationBounds;
    const xRange = xMax - xMin; // height range
    const yRange = yMax - yMin; // width range

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

    // Loop through all robots and draw markers for each with unique color
    robots.forEach((robot, idx) => {
      const robotX = parseFloat(robot?.robotXlocation);
      const robotY = parseFloat(robot?.robotYlocation);

      // Assign a color based on robot index, or cycle through the predefined colors
      const robotColor = robotColors[idx % robotColors.length];  // Use modulus for cycling through colors

      if (
        typeof robotX === 'number' &&
        typeof robotY === 'number' &&
        robotX >= xMin && robotX <= xMax &&
        robotY >= yMin && robotY <= yMax
      ) {
        const markerX = ((robotY - yMin) / yRange) * width;
        const markerY = ((robotX - xMin) / xRange) * height;

        // Draw the robot's marker
        ctx.beginPath();
        ctx.arc(markerX, markerY, 8, 0, 2 * Math.PI);
        ctx.fillStyle = robotColor;  // Set the robot's unique color here
        ctx.fill();
        ctx.strokeStyle = 'black';
        ctx.lineWidth = 2;
        ctx.stroke();

        // If the mouse is over this robot's marker, set the hover state
        if (hoveredRobot && hoveredRobot.idx === idx) {
          // Draw a tooltip at the mouse position
          const tooltipText = `Robot ${idx + 1}: (${robotX.toFixed(1)}, ${robotY.toFixed(1)})`;
          ctx.fillStyle = 'white';
          ctx.fillRect(tooltipPosition.x, tooltipPosition.y, ctx.measureText(tooltipText).width + 10, 25);
          ctx.fillStyle = 'black';
          ctx.fillText(tooltipText, tooltipPosition.x + 5, tooltipPosition.y + 15);
        }
      }
    });

  }, [simulationBounds, gifObj.url, robots, hoveredRobot, tooltipPosition]);

  const handleCanvasClick = (e) => {
    if (!simulationBounds || !onSetRobotCoords) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const { xMin, xMax, yMin, yMax } = simulationBounds;
    const xRange = xMax - xMin; // height range
    const yRange = yMax - yMin; // width range

    const px = ((e.clientX - rect.left) / rect.width) * canvas.width;  // horizontal (width)
    const py = ((e.clientY - rect.top) / rect.height) * canvas.height; // vertical (height)

    // Map vertical pixel to simulation x (height)
    const simX = xMin + (py / canvas.height) * xRange;
    // Map horizontal pixel to simulation y (width)
    const simY = yMin + (px / canvas.width) * yRange;

    // Snap to lower left of the 0.1x0.1 grid square
    const snappedX = Math.floor((simX - xMin) / 0.1) * 0.1 + xMin;
    const snappedY = Math.floor((simY - yMin) / 0.1) * 0.1 + yMin;

    onSetRobotCoords(snappedX, snappedY);
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

export default GifWithGrid;
