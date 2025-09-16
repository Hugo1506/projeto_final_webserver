import React from 'react';
import "./CompassRose.css"

const averageRadians = (angles) => {
  if (angles.length === 0) return null;
  const sinSum = angles.reduce((sum, rad) => sum + Math.sin(rad), 0);
  const cosSum = angles.reduce((sum, rad) => sum + Math.cos(rad), 0);
  return Math.atan2(sinSum / angles.length, cosSum / angles.length);
};

const CompassRose = ({ gifsInSet, selectedSetSimId, selectedRobotFilter = 'all' }) => {
  const filteredGifs = selectedSetSimId === -1
    ? gifsInSet
    : gifsInSet.filter(gif => gif.robotSim_id === selectedSetSimId);

  const robotPointsMap = {};

  filteredGifs.forEach(gif => {
    const simId = gif.robotSim_id ?? 'unknown';

    if (!Array.isArray(gif.robot_path)) return;

    const filteredPath = selectedRobotFilter === 'all'
      ? gif.robot_path
      : gif.robot_path.filter(p => String(p.robot) === String(selectedRobotFilter));

    filteredPath.forEach(point => {
      if (!point.robot_position) return;

      const key = `${point.robot}_sim_${simId}`;
      if (!robotPointsMap[key]) robotPointsMap[key] = [];

      robotPointsMap[key].push({
        iteration: point.iteration,
        position: point.robot_position,
        robot: point.robot,
      });
    });
  });

  const robotDirectionsMap = {};

  Object.entries(robotPointsMap).forEach(([key, points]) => {
    points.sort((a, b) => a.iteration - b.iteration);
    const robotId = points[0]?.robot;

    for (let i = 1; i < points.length; i++) {
      const prev = points[i - 1].position;
      const curr = points[i].position;
      if (!prev || !curr) continue;

      const dx = curr.x - prev.x;
      const dy = curr.y - prev.y;

      // Ignore huge jumps (likely between simulations)
      const distSq = dx * dx + dy * dy;
      if (distSq > 10) continue;

      const angleRad = Math.atan2(dx, dy); // Notice: dx and dy switched for Y-up "UP" orientation

      if (!robotDirectionsMap[robotId]) robotDirectionsMap[robotId] = [];
      robotDirectionsMap[robotId].push(angleRad);
    }
  });

  const robotAvgDirections = {};
  Object.entries(robotDirectionsMap).forEach(([robotId, angles]) => {
    const avgRad = averageRadians(angles);
    robotAvgDirections[robotId] = avgRad;
  });

  const SingleCompass = ({ robotId, angleRad }) => {
    const size = 200;
    const center = size / 2;
    const arrowLength = 40;

    const x2 = center + arrowLength * Math.sin(angleRad);
    const y2 = center - arrowLength * Math.cos(angleRad);


    return (
      <div className="single-compass">
        <svg width={size} height={size} className="compass-svg">
          <circle cx={center} cy={center} r={center - 5} stroke="#333" strokeWidth={2} fill="white" />
          <text x={center} y={center - (center - 30)} textAnchor="middle" fontWeight="bold">UP</text>
          <text x={center} y={center + (center - 15)} textAnchor="middle" fontWeight="bold">DOWN</text>
          <text x={center + (center - 30)} y={center + 5} textAnchor="middle" fontWeight="bold">RIGHT</text>
          <text x={center - (center - 30)} y={center + 5} textAnchor="middle" fontWeight="bold">LEFT</text>

          <line
            x1={center}
            y1={center}
            x2={x2}
            y2={y2}
            stroke="red"
            strokeWidth={2}
            markerEnd="url(#arrowhead)"
          />
          <defs>
            <marker
              id="arrowhead"
              markerWidth="10"
              markerHeight="7"
              refX="0"
              refY="3.5"
              orient="auto"
            >
              <polygon points="0 0, 10 3.5, 0 7" fill="red" />
            </marker>
          </defs>
        </svg>
        <div className="robot-label">Robot {robotId}</div>
        <div className="angle-label">Avg Direction: {angleRad.toFixed(2)} rad</div>
      </div>
    );
  };

  return (
    <div style={{ display: 'flex', flexWrap: 'wrap', justifyContent: 'center' }}>
      {Object.keys(robotAvgDirections).length > 0
        ? Object.entries(robotAvgDirections).map(([robotId, angleRad]) => (
          <SingleCompass key={robotId} robotId={robotId} angleRad={angleRad} />
        ))
        : <p>No robot directions available.</p>
      }
    </div>
  );
};

export default CompassRose;
