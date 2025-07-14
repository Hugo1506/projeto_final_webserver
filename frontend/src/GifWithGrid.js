import React, { useEffect, useRef, useState } from 'react';

const GifWithGrid = ({ gifObj, simulationBounds, robots, onSetRobotCoords,selectedRobotIdx, numberOfRobots,  grid, type }) => {
  const canvasRef = useRef(null);
  const [finalInputFlag, setFinalInputFlag] = useState(false);
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

    robots.forEach((robot, idx) => {
      if (idx < numberOfRobots){
        const robotX = parseFloat(robot?.robotXlocation);
        const robotY = parseFloat(robot?.robotYlocation);
        let finalRobotX = parseFloat(robot?.finalRobotXlocation);
        let finalRobotY = parseFloat(robot?.finalRobotYlocation);

        if(type != "linear"){
          finalRobotX = finalRobotY = null;
        }

        const robotColor = robotColors[idx % robotColors.length];

        if (
          typeof robotX === 'number' &&
          typeof robotY === 'number' &&
          robotX >= xMin && robotX <= xMax &&
          robotY >= yMin && robotY <= yMax
        ) {
          const markerX = ((robotY - yMin) / yRange) * width;
          const markerY = ((robotX - xMin) / xRange) * height;

          // Draw the robot's initial position marker
          ctx.beginPath();
          ctx.arc(markerX, markerY, 8, 0, 2 * Math.PI);
          ctx.fillStyle = robotColor;
          ctx.fill();
          ctx.strokeStyle = 'black';
          ctx.lineWidth = 2;
          ctx.stroke();

          ctx.font = "20px arial bold";
          ctx.fillStyle = "black";
          ctx.fillText("R" + (idx + 1), markerX + 20, markerY - 20);

          if (typeof finalRobotX === 'number' && typeof finalRobotY === 'number') {
            const finalMarkerX = ((finalRobotY - yMin) / yRange) * width;
            const finalMarkerY = ((finalRobotX - xMin) / xRange) * height;

            // Draw final robot position with 50% opacity
            ctx.beginPath();
            ctx.arc(finalMarkerX, finalMarkerY, 8, 0, 2 * Math.PI);
            ctx.fillStyle = robotColor; 
            ctx.fill();
            ctx.strokeStyle = 'black';
            ctx.lineWidth = 2;
            ctx.stroke();

            ctx.font = "20px arial bold";
            ctx.fillStyle = "black";
            ctx.fillText("R" + (idx + 1) + " Final", finalMarkerX + 20, finalMarkerY - 20);
          }
        }
      }  
    });
  }, [simulationBounds, gifObj.url, robots, grid]);

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

    // Check if robot simulation mode is linear
    if (type === 'linear') {
      if (!finalInputFlag) {
        onSetRobotCoords(snappedX, snappedY, null, null);
        setFinalInputFlag(true);
      } else {
        onSetRobotCoords(null, null, snappedX, snappedY);
        setFinalInputFlag(false);
      }
    } else {
      // If not in linear mode, just set initial coordinates
      onSetRobotCoords(snappedX, snappedY, null, null);
    }
  };

  useEffect(() => {
    if (finalInputFlag) {
      const canvas = canvasRef.current;
      const ctx = canvas.getContext('2d');
      const width = canvas.width;
      const height = canvas.height;
      const { xMin, xMax, yMin, yMax } = simulationBounds;
      const xRange = xMax - xMin;
      const yRange = yMax - yMin;

      const snappedX = robots[selectedRobotIdx]?.finalRobotXlocation;
      const snappedY = robots[selectedRobotIdx]?.finalRobotYlocation;

      const markerX = ((snappedY - yMin) / yRange) * width;
      const markerY = ((snappedX - xMin) / xRange) * height;

      const robotColor = robotColors[selectedRobotIdx % robotColors.length];

    
      ctx.beginPath();
      ctx.arc(markerX, markerY, 8, 0, 2 * Math.PI);
      ctx.fillStyle = `rgba(${parseInt(robotColor.substring(1, 3), 16)}, 
                              ${parseInt(robotColor.substring(3, 5), 16)}, 
                              ${parseInt(robotColor.substring(5, 7), 16)})`; 
      ctx.fill();
      ctx.strokeStyle = 'black';
      ctx.lineWidth = 2;
      ctx.stroke();
    }
  }, [finalInputFlag, robots, selectedRobotIdx]);

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
