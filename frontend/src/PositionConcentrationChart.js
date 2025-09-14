import React, { useMemo } from 'react';
import { Scatter } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  Title,
  Tooltip,
  Legend,
  PointElement,
  LinearScale
} from 'chart.js';

ChartJS.register(Title, Tooltip, Legend, PointElement, LinearScale);

function concentrationToColor(concentration, min, max) {
  if (concentration === 0) return 'blue';
  if (max === min) return 'hsl(120, 100%, 50%)'; 
  const norm = Math.max(0, Math.min(1, (concentration - min) / (max - min)));
  const r = norm < 0.5 ? norm * 2 * 255 : 255;
  const g = norm < 0.5 ? 255 : 255 - (norm - 0.5) * 2 * 255;
  return `rgb(${Math.round(r)},${Math.round(g)},0)`;
}

const PositionConcentrationChart = ({ gifsInSet, selectedSetSimId, currentIteration, selectedRobotFilter, showTotalStatsRobotSim, simulationBounds }) => {
  const simGifs = gifsInSet.filter(g => g.robotSim_id === selectedSetSimId);

  const points = useMemo(() => (
    (showTotalStatsRobotSim
      ? simGifs.filter(gifObj => Array.isArray(gifObj.robot_path))
      : simGifs.filter(gifObj => gifObj.iteration <= currentIteration)
    )
      .flatMap(gifObj =>
        (gifObj.robot_path || [])
          .filter(point => selectedRobotFilter === 'all' ? true : String(point.robot) === String(selectedRobotFilter))
          .map(point => ({
            x: point.robot_position.x,
            y: point.robot_position.y,
            concentration: Number(point.concentration),
            robot: point.robot,
            iteration: gifObj.iteration
          }))
      )
  ), [simGifs, currentIteration, selectedRobotFilter, showTotalStatsRobotSim]);

  const concentrations = points.map(p => p.concentration);
  const minC = Math.min(...concentrations, 0);
  const maxC = Math.max(...concentrations, 1);

  const markers = points.filter(p => p.concentration > 0);

    const data = {
    datasets: [
        {
        label: 'Robot Positions',
        data: points
            .filter(p => p.iteration === currentIteration)
            .map(p => ({ x: p.x, y: p.y, robot: p.robot, concentration: p.concentration })),
        pointBackgroundColor: points
            .filter(p => p.iteration === currentIteration)
            .map(p => concentrationToColor(p.concentration, minC, maxC)),
        pointRadius: 6,
        },
        {
        label: 'Concentration Markers',
        data: markers.map(p => ({ x: p.x, y: p.y, robot: p.robot, concentration: p.concentration })),
        pointBackgroundColor: markers.map(p => concentrationToColor(p.concentration, minC, maxC)),
        pointRadius: 3,
        }
    ]
    };

    const options = {
    responsive: true,
    plugins: {
        legend: { display: false },
        tooltip: {
        callbacks: {
            label: ctx => {
            const p = ctx.dataset.data[ctx.dataIndex];
            return `Robot ${p.robot}: (x:${p.x.toFixed(2)}, y:${p.y.toFixed(2)}) Conc:${p.concentration.toFixed(4)}`;
            }
        }
        }
    },
    scales: {
        x: { 
        title: { display: true, text: 'X Position' },
        min: simulationBounds?.xMin ?? undefined,
        max: simulationBounds?.xMax ?? undefined
        },
        y: { 
        title: { display: true, text: 'Y Position' },
        min: simulationBounds?.yMin ?? undefined,
        max: simulationBounds?.yMax ?? undefined
        }
    }
    };


  return (
    <div style={{ background: '#fff', padding: '10px', borderRadius: '8px', boxShadow: '0 0 8px #eee' }}>
      <Scatter data={data} options={options} />
      <div style={{ textAlign: 'center', marginTop: '8px', fontSize: '13px', color: '#555' }}>
        Robot positions colored by concentration (blue = 0, green→yellow→red for bigger than 0)
      </div>
    </div>
  );
};

export default PositionConcentrationChart;