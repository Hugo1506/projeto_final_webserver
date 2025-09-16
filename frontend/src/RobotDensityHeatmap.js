import React, { useEffect, useState } from 'react';
import { Chart as ChartJS, LinearScale, Tooltip, Title } from 'chart.js';
import { MatrixController, MatrixElement } from 'chartjs-chart-matrix';
import { Chart } from 'react-chartjs-2';
import { binRobotPositions } from './RobotHeatmapUtils'; 
import './RobotDensityHeatmap.css';

ChartJS.register(LinearScale, Tooltip, Title, MatrixController, MatrixElement);

const RobotDensityHeatmap = ({ gifsInSet, selectedSetSimId, simulationBounds }) => {
  const [canvasSize, setCanvasSize] = useState({ width: 500, height: 500 });

  const filteredPositions = gifsInSet
  .filter(gif =>
    selectedSetSimId === -1
      ? gif.robotSim_id !== undefined && gif.robotSim_id !== null
      : gif.robotSim_id === selectedSetSimId
  )
  .flatMap(gif =>
    (gif.robot_path || [])
      .map(r => r.robot_position)
      .filter(
        pos =>
          pos &&
          typeof pos.x === 'number' &&
          typeof pos.y === 'number' &&
          !isNaN(pos.x) &&
          !isNaN(pos.y)
      )
  );

  const xValues = filteredPositions.map(p => p.x);
  const yValues = filteredPositions.map(p => p.y);


  const binSize = 0.1;
  const binnedData = binRobotPositions(filteredPositions, binSize);

    const matrixData = binnedData.map(bin => ({
    x: bin.x + binSize / 2,
    y: bin.y + binSize / 2,
    v: bin.v,
    width: binSize,
    height: binSize,
    }));


  useEffect(() => {
    const maxWidth = 800; 
    const canvasWidth = Math.min((simulationBounds.xMax - simulationBounds.xMin) * 40, maxWidth); 
    const canvasHeight = (simulationBounds.yMax - simulationBounds.yMin) * 40;
    setCanvasSize({ width: canvasWidth, height: canvasHeight });
  }, [simulationBounds.xMin, simulationBounds.xMax, simulationBounds.yMin, simulationBounds.yMax]);

    const maxV = Math.max(...matrixData.map(d => d.v), 1);

    function getColor(value) {
    const t = Math.min(1, value / maxV); 

    if (t <= 0.33) {
        const ratio = t / 0.33;
        const r = 0 + ratio * (0 - 0);
        const g = 0 + ratio * (255 - 0);
        const b = 255 + ratio * (0 - 255);
        return `rgb(${Math.round(r)},${Math.round(g)},${Math.round(b)})`;
    } else if (t <= 0.66) {
        const ratio = (t - 0.33) / 0.33;
        const r = 0 + ratio * 255;
        const g = 255;
        const b = 0;
        return `rgb(${Math.round(r)},${Math.round(g)},${Math.round(b)})`;
    } else {
        const ratio = (t - 0.66) / 0.34;
        const r = 255;
        const g = 255 - ratio * 255;
        const b = 0;
        return `rgb(${Math.round(r)},${Math.round(g)},${Math.round(b)})`;
    }
    }


  if (filteredPositions.length === 0) {
    return <p>No robot position data available for this simulation.</p>;
  }

    const data = {
    datasets: [
        {
        label: 'Robot Density Heatmap',
        data: matrixData,
        backgroundColor: ctx => getColor(ctx.raw.v),
        borderWidth: 0,
        },
    ],
    };

    const options = {
        maintainAspectRatio: false,
        responsive: true,
        plugins: {
            title: {
            display: true,
            text: 'Robot Density Heatmap',
            },
            tooltip: {
            callbacks: {
                label: ctx => {
                const { x, y, v } = ctx.raw;
                return `x: ${x.toFixed(2)}, y: ${y.toFixed(2)} → count: ${v}`;
                },
            },
            },
        },
        scales: {
            x: {
            title: { display: true, text: 'X Position' },
            min: simulationBounds.xMin,
            max: simulationBounds.xMax,
            ticks: { stepSize: binSize },
            },
            y: {
            title: { display: true, text: 'Y Position' },
            min: simulationBounds.yMin,
            max: simulationBounds.yMax,
            ticks: { stepSize: binSize },
            },
        },
        };


  return (
    <div className="heatmap-container" style={{ height: '500px', width: '500px', overflow: 'auto' }}>
      <Chart type="matrix" data={data} options={options} />
    </div>
  );
};

export default RobotDensityHeatmap;
