import React from "react";
import { Bar } from "react-chartjs-2";
import { Chart as ChartJS, CategoryScale, LinearScale, BarElement, Title, Tooltip, Legend } from "chart.js";

ChartJS.register(CategoryScale, LinearScale, BarElement, Title, Tooltip, Legend);

const SimpleBarChart = ({ data }) => {
  const maxConcentration = Math.max(...data.map(d => d.concentration), 1);
  const chartData = {
    labels: data.map(d => `R${d.robot}`),  
    datasets: [
      {
        label: "Concentration",
        data: data.map(d => d.concentration), 
        backgroundColor: "#ff0000", 
        borderRadius: 5,
        borderColor: "#ff0000",
        borderWidth: 1,
      }
    ],
  };

  const chartOptions = {
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      x: {
        beginAtZero: true,
      },
      y: {
        beginAtZero: true,
        max: maxConcentration,
        ticks: {
          callback: function (value) {
            return value.toFixed(6);
          },
        },
      },
    },
    plugins: {
      tooltip: {
        callbacks: {
          label: function (tooltipItem) {
            return `Robot ${data[tooltipItem.dataIndex].robot}\nConcentration: ${tooltipItem.raw.toFixed(6)}`;
          },
        },
      },
    },
  };

  return (
    <div className="bar-chart-container">
      <Bar data={chartData} options={chartOptions} />
    </div>
  );
};

export default SimpleBarChart;
