const express = require('express');
const bodyParser = require('body-parser');
const mysql = require('mysql2');
const bcrypt = require('bcrypt');
const multer = require('multer');
const path = require('path');
const crypto = require('crypto');
const jwt = require('jsonwebtoken');
const cors = require('cors');
const fs = require('fs');
const yaml = require('js-yaml');
const zlib = require('zlib');
const axios = require('axios');


const app = express();
const port =3000;
const saltRounds = 10;
const secretKey = '8711aff2c33a868742a1122a185e1ecb6b17beb1a4c4097a9cc37092b8fddb20fb5049c81b07dcbf979e678312c039d1939824efbefebdacf94bac0ba1421f5';

app.use(cors());
app.use(bodyParser.json({ limit: '50mb' }));
app.use(bodyParser.urlencoded({ limit: '50mb', extended: true }));

// Set up the MySQL connection pool
const pool = mysql.createPool({
  connectionLimit: 10,
  host: '127.0.0.1',
  user: 'hugo',
  password: 'hugo',
  database: 'projeto_final'
});

const storage = multer.diskStorage({
  destination: function (req, file, cb) {
    const username = req.body.username;
    const simulationNumber = req.body.simulationNumber;
    let baseDir = `simulations/${username}/${username}_${simulationNumber}`;

    if (file.fieldname === 'innerCadFiles' || file.fieldname === 'outerCadFiles') {
      baseDir = `${baseDir}/CADs`;
    } else if (file.fieldname === 'windFiles') {
      baseDir = `${baseDir}/windsim`;
    }

    fs.mkdirSync(baseDir, { recursive: true });

    cb(null, baseDir);
  },

  filename: function (req, file, cb) {
    const username = req.body.username;
    const simulationNumber = req.body.simulationNumber;
    const ext = path.extname(file.originalname);
    const randomString = crypto.randomBytes(8).toString('hex');
    let filename;

    if (file.fieldname === 'windFiles') {
      const windFileIndex = req.body.windFileIndex || 0; 
      filename = `${username}_${simulationNumber}_${randomString}_${windFileIndex}${ext}`;

      req.body.windFileIndex = (req.body.windFileIndex || 0) + 1;
    } else {
      filename = `${username}_${simulationNumber}_${randomString}${ext}`;
    }

    cb(null, filename);
  }
});
const upload = multer({ storage: storage });

// Função que envia os dados para o volume que os dois containers partilham
function sendToVolume(username, simulationNumber, innerCadFilePaths, outerCadFilePaths, windFilePaths) {
  const baseDir = path.join('/simulation_data', username, `sim_${simulationNumber}`);

    // define os subdiretórios
    const cadDir = path.join(baseDir, 'cad_models');
    const windDir = path.join(baseDir, 'wind_simulations');
    const paramsDir = path.join(baseDir, 'params');

    // cria os diretórios caso eles não existam
    [cadDir, windDir, paramsDir].forEach(dir => {
        if (!fs.existsSync(dir)) {
            fs.mkdirSync(dir, { recursive: true });
        }
    });

    // copia os ficheiros CAD para o local onde o gaden vai ler
    innerCadFilePaths.forEach((cadFilePath) => {
      const fileName = path.basename(cadFilePath);
      const destPath = path.join(cadDir, fileName);
      fs.copyFileSync(cadFilePath, destPath);
    });
  
    outerCadFilePaths.forEach((cadFilePath) => {
      const fileName = path.basename(cadFilePath);
      const destPath = path.join(cadDir,fileName); 
      fs.copyFileSync(cadFilePath, destPath);
    });

    // copia a simulação de vento para o local onde o gaden vai ler
    windFilePaths.forEach((windFilePath) => {
      const fileName = path.basename(windFilePath); 
      const windDestPath = path.join(windDir, fileName); 
      fs.copyFileSync(windFilePath, windDestPath);
    });

    const params = {
      gaden_preprocessing:{
        ros__parameters: {
          cell_size: 0.1,
          models: innerCadFilePaths.filter(filePath => filePath.endsWith('.stl')).map(filePath => {
            return `$(var pkg_dir)/scenarios/$(var scenario)/cad_models/${path.basename(filePath)}`;
          }),
          outlets_models: outerCadFilePaths.filter(filePath => filePath.endsWith('.stl')).map(filePath => {
            return `$(var pkg_dir)/scenarios/$(var scenario)/cad_models/${path.basename(filePath)}`;
          }),
          empty_point_x: (1.0).toFixed(1),
          empty_point_y: (1.0).toFixed(1),
          empty_point_z: (0.5).toFixed(1),
          uniformWind: false,
          wind_files: `$(var pkg_dir)/scenarios/$(var scenario)/wind_simulations/$(var wind_sim_path)`,
          output_path: `$(var pkg_dir)/scenarios/$(var scenario)`
        }
      }
    };
  
    const gaden_params ={ 
      gaden_environment: {
        ros__parameters: {
          verbose: false,
          wait_preprocessing: false,
          fixed_frame: "map",
        }
      },
      gaden_filament_simulator: {
        ros__parameters: {
          verbose: false,
          wait_preprocessing: false,
          sim_time: "$(var sim_time)", 
          time_step: "$(var time_step)", 
          num_filaments_sec: "$(var num_filaments_sec)",
          variable_rate: "$(var variable_rate)", 
          filament_stop_steps: "$(var filament_stop_steps)",
          ppm_filament_center: "$(var ppm_filament_center)", 
          filament_initial_std: "$(var filament_initial_std)", 
          filament_growth_gamma: "$(var filament_growth_gamma)", 
          filament_noise_std: "$(var filament_noise_std)", 
          gas_type: "$(var gas_type)", 
          temperature: "$(var temperature)",
          pressure: "1.0", 
          concentration_unit_choice: 1, 
          occupancy3D_data: "$(var pkg_dir)/scenarios/$(var scenario)/OccupancyGrid3D.csv",
          fixed_frame: "map",

          wind_data: "$(var pkg_dir)/scenarios/$(var scenario)/wind_simulations/$(var wind_sim_path)",
          wind_time_step: "1.0", 

          allow_looping: true,
          loop_from_step: 0,
          loop_to_step: 24,

        
          source_position_x: "1.0", // (m)
          source_position_y: "1.0", // (m)
          source_position_z: "1.0", // (m)

          save_results: 1, 
          results_time_step: 0.5, 
          results_min_time: "0.0", 
          results_location: "$(var pkg_dir)/scenarios/$(var scenario)/gas_simulations/$(var simulation)"
        }
      },
      gaden_player: {
        ros__parameters: {
          verbose: false,
          player_freq: "2.0", // (Hz) Freq para carregar os arquivos de log da simulação
          initial_iteration: 0,
          num_simulators: 1, // Número de simulações para carregar [1-inf] (útil para múltiplas fontes e gases)

          // Dados do pacote "filament_simulator"
          simulation_data_0: "$(var pkg_dir)/scenarios/$(var scenario)/gas_simulations/$(var simulation)",
          occupancyFile: "$(var pkg_dir)/scenarios/$(var scenario)/OccupancyGrid3D.csv",

          // Loop options
          allow_looping: false,
          loop_from_iteration: 15,
          loop_to_iteration: 24
        }
      }
    };
    
    const innerDaeFiles = innerCadFilePaths.filter(filePath => filePath.endsWith('.dae'));
    const outerDaeFiles = outerCadFilePaths.filter(filePath => filePath.endsWith('.dae'));

    innerDaeFiles.forEach((cadFilePath, index) => {
      const key = `CAD_${index}`;
      gaden_params.gaden_environment.ros__parameters[key] = `$(var pkg_dir)/scenarios/$(var scenario)/cad_models/${path.basename(cadFilePath)}`;
      
      gaden_params.gaden_environment.ros__parameters[`${key}_color`] = [0.92, 0.96, 0.96]; 
    });
    
    outerDaeFiles.forEach((cadFilePath, index) => {
      const key = `CAD_${innerDaeFiles.length + index}`;
      gaden_params.gaden_environment.ros__parameters[key] = `$(var pkg_dir)/scenarios/$(var scenario)/cad_models/${path.basename(cadFilePath)}`;
      
      gaden_params.gaden_environment.ros__parameters[`${key}_color`] = [0.96, 0.17, 0.3]; 
    });

    const yamlSimFilePath = path.join(paramsDir, 'gaden_params.yaml');

    const yamlSimContent = yaml.dump(gaden_params, {
      lineWidth: 1000, // Evitar que o YAML seja quebrado em várias linhas
      noRefs: true, // Garante que as referências não sejam geradas
      pretty: true,    // Optional: Ensures a readable format, but still inline arrays
      flowLevel: 1 
    });

    fs.writeFileSync(yamlSimFilePath, yamlSimContent, 'utf8');

    // Guardar o YAML gerado em um arquivo
    const yamlPreprocFilePath = path.join(paramsDir, 'preproc_params.yaml');

    

    const yamlPreprocContent = yaml.dump(params, {
      lineWidth: 1000, // Evitar que o YAML seja quebrado em várias linhas
      noRefs: true,    // Garante que as referências não sejam geradas
    });

    fs.writeFileSync(yamlPreprocFilePath, yamlPreprocContent, 'utf8');


}

app.get('/getSimulations', (req, res) => {
  const username = req.query.username;

  if (!username) {
    return res.status(400).json({ error: 'Username is required' });
  }

  const queryGetSimulations = 'SELECT simulationName, simulation FROM simulation_queue WHERE username = ? AND status != \'DELETED\' AND status != \'OUT OF BOUNDS\'';

  pool.query(queryGetSimulations, [username], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).json({ error: 'Internal Server Error' });
    }

    if (results.length === 0) {
      return res.status(404).json({ message: 'No simulations found for this username' });
    }

    const simulations = results.map(result => ({
      simulationName: result.simulationName,
      simulation: result.simulation
    }));

    res.status(200).json({ simulations });
  });
});


app.get('/getIntialWindFrame', (req,res) => {
  
});

app.post('/uploadSimulationResults', (req, res) => {
  const simulation = req.body.simulation;
  const simulationResultType = req.body.type;
  const height = req.body.height;
  const compressedGif = req.body.gif;
  const iteration = req.body.iteration;
  const time = req.body.time;
  const robotSim_id = req.body.robotSim_id !== undefined ? req.body.robotSim_id : -1;

  const queryInsertSimulationResult = 'INSERT INTO simulation_results (simulation,type,gif,height,iteration, time, robotSim_id) VALUES (?, ?, ?, ?, ?, ?, ?)';

  pool.query(queryInsertSimulationResult, [simulation, simulationResultType, compressedGif,height, iteration, time, robotSim_id], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    res.status(200).json({
      id: results.insertId,
    });
  });

});


app.post('/setBounds', async (req, res) => {
  const simulation = req.body.simulation;
  const xMin = req.body.x_min;
  const xMax = req.body.x_max;
  const yMin = req.body.y_min;
  const yMax = req.body.y_max;
  const zMin = req.body.z_min;
  const zMax = req.body.z_max;

  const username = simulation.split('_')[0];
  const simulationNumber = simulation.split('_')[1];
  await axios.get('http://simulation:8000/example_simulation', {
    params: {
      username,
      simulationNumber,
      zMin,
      zMax
    }
  });

  try {
    const result = await setStatusToInSimulation(simulation);
  } catch (error) {
    console.error(error);
    res.status(500).send(error);
  }


  const bounds = JSON.stringify({ xMin, xMax, yMin, yMax, zMin, zMax });

  const querySetSimulationBounds = `UPDATE simulation_queue 
      SET simulation_bounds = ? 
      WHERE simulation = ?`
    
  
    pool.query(querySetSimulationBounds, [bounds, simulation], (error, results) => {
      if (error) {
        console.error(error);
        return res.status(500).send('Internal Server Error');
      }

      if (results.affectedRows === 0) {
        return res.status(404).send('Simulation not found');
      }

      
      res.status(200).json({
        message: 'Results uploaded successfully',
      });
  });
});

app.get('/getBoundsValues', (req, res) => {
  const simulation = req.query.simulation;
  const queryGetBounds = 'SELECT simulation_bounds FROM simulation_queue WHERE simulation = ?';

  pool.query(queryGetBounds, [simulation], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).json({ error: 'Internal Server Error' });
    }

    if (results.length === 0) {
      return res.status(404).json({ message: 'Simulation not found' });
    }

    const boundsString = results[0].simulation_bounds;
    let bounds;

    try {
      bounds = JSON.parse(boundsString);
    } catch (err) {
      console.error('Failed to parse simulation_bounds JSON:', err);
      return res.status(500).json({ error: 'Invalid bounds data' });
    }

    if (bounds){
      const { xMin, xMax, yMin, yMax, zMin, zMax } = bounds; 
      
      res.status(200).json({ xMin, xMax, yMin, yMax, zMin, zMax });
      
    }else{
      res.status(500).send("bounds is null")
    }

    
  });

});


app.get('/getRobotSimulationID', (req, res) => {
  const simulation = req.query.simulation;
  const queryGetRobotSimulationID = 'SELECT MAX(robotSim_id)  FROM simulation_results WHERE simulation = ? AND type = "robot"';



  pool.query(queryGetRobotSimulationID, [simulation], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).json({ error: 'Internal Server Error' });
    }

    if (results.length === 0) {
      const id = 0;
      return res.status(200).json({ id });
    }

    const id = results[0]['MAX(robotSim_id)']; 
    res.status(200).json({ id });
  });
});


app.post('/silkworm_moth_simulation', async (req, res) => {
    const { username, simulation, height, robots, startingIteration, nameOfSet, numOfSim, deviation} = req.body;

    // Validate required parameters
    if (!username || !simulation || !height || !robots) {
      return res.status(400).json({ 
        error: 'Missing required parameters. Need username, simulation, height, and robots.'
      });
    }

    const simulationNumber = simulation.toString().split('_')[1];
    const numberOfRobots = robots.length;
    res.status(202).json({ message: 'Simulation started. Processing in background.' });

    (async () => {
    for (let currentSimulationNumber  = 1; currentSimulationNumber< Number(numOfSim)+1; currentSimulationNumber++){
      const simulationSet = nameOfSet + "/" + currentSimulationNumber+"/"+numOfSim;
      console.log(`Running simulation number: ${currentSimulationNumber} / ${numOfSim}`);
      try {
        const response = await axios.get('http://simulation:8000/silkworm_moth_simulation', {
          params: {
            username,
            simulationNumber,
            height,
            numberOfRobots,
            startingIteration,
            deviation,
            robots: JSON.stringify(robots)
          }
        });

        const message = response.data;
        const frames = message.frames;
        const robotSim_id = message.robotSim_id;

        const framesByIteration = {};
        frames.forEach(frame => {
          if (!framesByIteration[frame.iteration]) {
            framesByIteration[frame.iteration] = [];
          }
          framesByIteration[frame.iteration].push(frame);
        });

            const queries = Object.entries(framesByIteration).map(([iteration, framesArray]) => {
              const robotPath = JSON.stringify(framesArray);
              return new Promise((resolve, reject) => {
                pool.query(
                  `UPDATE simulation_results 
                  SET robot_path = ?, simulation_set = ?
                  WHERE simulation = ? 
                  AND type = 'robot'
                  AND robotSim_id = ?
                  AND iteration = ?`,
                  [robotPath, simulationSet, simulation, robotSim_id, iteration],
                  (error, results) => {
                    if (error) reject(error);
                    else resolve(results);
                  }
                );
              });
            });

            await Promise.all(queries);

            console.log(`Robot simulation for ${simulation} completed.`);
          } catch (error) {
            console.error('Background robotSimulation error:', error);
          }
      }
    })();
  }
);


app.post('/pso_simmulation', async (req, res) => {
  const { username, simulation, height, robots, startingIteration, nameOfSet, numOfSim, deviation, useRos } = req.body;
  console.log(useRos);

  const numberOfRobots = robots.length;
  const simulationNumber = simulation.split('_')[1];
  res.status(202).json({ message: 'Simulation started. Processing in background.' });
  
  (async () => {
  for (let currentSimulationNumber  = 1; currentSimulationNumber< Number(numOfSim)+1; currentSimulationNumber++){
    const simulationSet = nameOfSet + "/" + currentSimulationNumber+"/"+numOfSim;
    console.log(`Running simulation number: ${currentSimulationNumber} / ${numOfSim}`);
    try {
      const response = await axios.get('http://simulation:8000/pso_simmulation', {
        params: {
          username,
          simulationNumber,
          height,
          numberOfRobots,
          startingIteration,
          deviation,
          useRos,
          robots: JSON.stringify(robots)
        }
      });

      const message = response.data;
      const frames = message.frames;
      const robotSim_id = message.robotSim_id;

      const framesByIteration = {};
      frames.forEach(frame => {
        if (!framesByIteration[frame.iteration]) {
          framesByIteration[frame.iteration] = [];
        }
        framesByIteration[frame.iteration].push(frame);
      });


        const queries = Object.entries(framesByIteration).map(([iteration, framesArray]) => {
          const robotPath = JSON.stringify(framesArray);
          return new Promise((resolve, reject) => {
            pool.query(
              `UPDATE simulation_results 
              SET robot_path = ?, simulation_set = ?
              WHERE simulation = ? 
              AND type = 'robot'
              AND robotSim_id = ?
              AND iteration = ?`,
              [robotPath, simulationSet, simulation, robotSim_id, iteration],
              (error, results) => {
                if (error) reject(error);
                else resolve(results);
              }
            );
          });
        });

        await Promise.all(queries);

        console.log(`Robot simulation for ${simulation} completed.`);
      } catch (error) {
        console.error('Background robotSimulation error:', error);
      }
      }
    })();
  }
);




app.post('/robotSimulation', async (req, res) => {
  const { username, simulation, height, robots, startingIteration, nameOfSet, numOfSim, deviation} = req.body;
  const numberOfRobots = robots.length;
  const simulationNumber = simulation.split('_')[1];
  

  res.status(202).json({ message: 'Simulation started. Processing in background.' });
  
    (async () => {
      
      for (let currentSimulationNumber  = 1; currentSimulationNumber< Number(numOfSim)+1; currentSimulationNumber++){
        var startTime = performance.now()
        const simulationSet = nameOfSet + "/" + currentSimulationNumber+"/"+numOfSim;
        console.log(`Running simulation number: ${currentSimulationNumber} / ${numOfSim}`);
        try {
          const response = await axios.get('http://simulation:8000/robot_simulation', {
            params: {
              username,
              simulationNumber,
              height,
              numberOfRobots,
              startingIteration,
              deviation,
              robots: JSON.stringify(robots)
            }
          });

          const message = response.data;
          const frames = message.frames;
          const robotSim_id = message.robotSim_id;

          const framesByIteration = {};
          frames.forEach(frame => {
            if (!framesByIteration[frame.iteration]) {
              framesByIteration[frame.iteration] = [];
            }
            framesByIteration[frame.iteration].push(frame);
          });

          const queries = Object.entries(framesByIteration).map(([iteration, framesArray]) => {
            const robotPath = JSON.stringify(framesArray);
            return new Promise((resolve, reject) => {
              pool.query(
                `UPDATE simulation_results 
                SET robot_path = ?, simulation_set = ?
                WHERE simulation = ? 
                AND type = 'robot'
                AND robotSim_id = ?
                AND iteration = ?`,
                [robotPath, simulationSet, simulation, robotSim_id, iteration],
                (error, results) => {
                  if (error) reject(error);
                  else resolve(results);
                }
              );
            });
          });

          await Promise.all(queries);
          var endTime = performance.now()

          console.log(`Robot simulation for ${simulation} completed.`);
          console.log(`took ${endTime - startTime} milliseconds`)
        } catch (error) {
          console.error('Background robotSimulation error:', error);
        }
      }
    })();
  }
);


app.post ('/uploadPlumeLocation', (req, res) => {
  const{username,
    simulationNumber,
    plumeXlocation,
    plumeYlocation,
    plumeZlocation,
    temperatureInK,
    ppmCenter,
    numFilamentsSec,
    filamentInitialStd,
    filamentGrowth,
    filamentNoise} = req.body;

  axios.get('http://simulation:8000/set_plume_location', {
    params: {
      username,
      simulationNumber,
      plumeXlocation,
      plumeYlocation,
      plumeZlocation,
      temperatureInK,
      ppmCenter,
      numFilamentsSec,
      filamentInitialStd,
      filamentGrowth,
      filamentNoise
    }

  })
  .then(response => {

   return res.status(200).json({ message: 'Plume location saved'});

  })
  .catch(error => {
    console.error('Error calling simulation service:', error.message);
    return res.status(500).json({ error: 'Failed to call simulation backend' });
  });
});



app.post('/deleteSimulationSet', (req, res) => {
  const set = req.query.set;
  const simulation = req.query.simulation;
  const nameOfSet = set.split('/')[0].trim();

  const query = `
    DELETE FROM simulation_results
    WHERE simulation = ?
      AND simulation_set LIKE CONCAT(?, '/%')
  `;

  pool.query(query, [simulation, nameOfSet], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }
    res.status(200).send('Simulation set removed successfully');
  });
});



// rota que apaga a simulação
app.post('/deleteSimulation', (req, res) => {
  const simulation = req.query.simulation;
  // extrai o nome de utilizador e o número da simulação 
  const [name, number] = simulation.split('_');

  // Define o caminho para a pasta da simulação
  const simulationDirPath = path.join('/simulation_data', name, `sim_${number}`);

  const queryUpdateSimulationStatus = 'UPDATE simulation_queue SET status = ? WHERE simulation = ?';
  
  pool.query(queryUpdateSimulationStatus, ['DELETED', simulation], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.affectedRows === 0) {
      return res.status(404).send('Simulation not found');
    }
  
    // remove a pasta da simulação recursivamente 
    fs.rm(simulationDirPath, { recursive: true, force: true }, (err) => {
      if (err) {
        console.error('Error deleting simulation directory:', err);
        return res.status(500).send('Failed to delete simulation directory');
      }

      res.status(200).send('Simulation removed successfully');
    });
  });
});


app.get('/getEnviromentFrames', (req, res) => {
  const simulation = req.query.simulation;

  const query = `
    SELECT id, gif, height, type, iteration
    FROM simulation_results
    WHERE simulation = ? AND type = 'plume' AND iteration = 0 
  `;

  pool.query(query, [simulation], async (error, results) => {
    if (error) {
      console.error('SQL error:', error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.length === 0) {
      console.log('No results found for simulation:', simulation);
      return res.status(404).send('Environment frames not found');
    }

    const uniqueHeights = new Set();
    const frames = [];

    try {
      const decompressedFrames = await Promise.all(
        results.map(result => {
          return new Promise((resolve, reject) => {
            const compressedGifBuffer = Buffer.from(result.gif, 'base64');

            zlib.unzip(compressedGifBuffer, (err, decompressedBuffer) => {
              if (err) return reject(err);

              if (!uniqueHeights.has(result.height)) {
                uniqueHeights.add(result.height);
                resolve({
                  gif: decompressedBuffer.toString('base64'),
                  height: result.height,
                  type: result.type,
                  iteration: result.iteration,
                });
              } else {
                resolve(null); 
              }
            });
          });
        })
      );

      const filteredFrames = decompressedFrames.filter(frame => frame !== null);

      if (filteredFrames.length === 0) {
        console.warn('No unique heights or all decompressions failed.');
        return res.status(500).send('Failed to process environment frames');
      }

      res.set('Content-Type', 'application/json');
      res.json(filteredFrames);
    } catch (err) {
      console.error('Decompression error:', err);
      res.status(500).send('Error decompressing GIF data');
    }
  });
});



app.get('/getGifsFromSimulation', (req, res) => {
  const raw_set = req.query.set;
  const simulation = req.query.simulation;

  set = raw_set.split('/')[0];
  console.log(set);
  const queryGetSimulationResults = `
      SELECT 
        id,
        gif, height, type, iteration, robotSim_id, robot_path
      FROM simulation_results
      WHERE simulation = ? AND simulation_set LIKE CONCAT(?, '/%')`;

  pool.query(queryGetSimulationResults, [simulation,set], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.length === 0) {
      return res.status(404).send('Simulation not found');
    }

    const gifs = [];
    let processedCount = 0;

    results.forEach((result, index) => {
      const compressedGifBase64 = result.gif;
      const compressedGifBuffer = Buffer.from(compressedGifBase64, 'base64');


      zlib.unzip(compressedGifBuffer, (err, decompressedBuffer) => {
        if (err) {
          console.error(`Error decompressing GIF at index ${index}:`, err);
          return res.status(500).send('Failed to decompress GIF');
        }

       var robotSim_id_to_send; 
       if (result.robotSim_id === null || result.robotSim_id === undefined) {
          robotSim_id_to_send = -1;
       }else{
          robotSim_id_to_send = result.robotSim_id;
       }

        gifs.push({
          gif: decompressedBuffer.toString('base64'),
          height: result.height,
          type: result.type,
          iteration: result.iteration,
          robotSim_id: robotSim_id_to_send,
          robot_path: result.robot_path
        });



        processedCount++;

        if (processedCount === results.length) {
          res.set('Content-Type', 'application/json'); 
          res.json(gifs); 
        }
      });
    });
  });
});

app.get('/getSimulationResultsGifs', (req, res) => {
  const simulation = req.query.simulation;
  const queryGetSimulationResults = `
      SELECT 
        id,
        gif, height, type, iteration, robotSim_id, robot_path
      FROM simulation_results
      WHERE simulation = ? AND type != 'robot'`;

  pool.query(queryGetSimulationResults, [simulation,simulation], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.length === 0) {
      return res.status(404).send('Simulation not found');
    }

    const gifs = [];
    let processedCount = 0;

    results.forEach((result, index) => {
      const compressedGifBase64 = result.gif;
      const compressedGifBuffer = Buffer.from(compressedGifBase64, 'base64');


      zlib.unzip(compressedGifBuffer, (err, decompressedBuffer) => {
        if (err) {
          console.error(`Error decompressing GIF at index ${index}:`, err);
          return res.status(500).send('Failed to decompress GIF');
        }

       var robotSim_id_to_send; 
       if (result.robotSim_id === null || result.robotSim_id === undefined) {
          robotSim_id_to_send = -1;
       }else{
          robotSim_id_to_send = result.robotSim_id;
       }

        gifs.push({
          gif: decompressedBuffer.toString('base64'),
          height: result.height,
          type: result.type,
          iteration: result.iteration,
          robotSim_id: robotSim_id_to_send,
          robot_path: result.robot_path
        });



        processedCount++;

        if (processedCount === results.length) {
          res.set('Content-Type', 'application/json'); 
          res.json(gifs); 
        }
      });
    });
  });
});

app.get('/getRobotSet', (req, res) => {
  const simulation = req.query.simulation;
  const queryGetSimulationResults = `
      SELECT 
        DISTINCT simulation_set,
        simulation
      FROM simulation_results
      WHERE simulation = ? AND type = 'robot'`;

  pool.query(queryGetSimulationResults, [simulation], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.length === 0) {
      return res.status(404).send('Set not found');
    }

    const sets = results.map(result => ({
      simulation_set: result.simulation_set,
      simulation: result.simulation
    }));

    res.json(sets); 
  });
});


// rota de upload de ficheiros
app.post('/uploadFiles', upload.fields([
  { name: 'innerCadFiles', maxCount: 10 },  
  { name: 'outerCadFiles', maxCount: 10 },  
  { name: 'windFiles', maxCount: 50 }   
]), async  (req, res) => {

  if (!req.files || 
      (!req.files.innerCadFiles || req.files.innerCadFiles.length === 0) || 
      (!req.files.outerCadFiles || req.files.outerCadFiles.length === 0) || 
      (!req.files.windFiles || req.files.windFiles.length === 0)) {
    return res.status(400).send('No files were uploaded. Please upload both CAD and wind files.');
  }

  const username = req.body.username;
  const simulationNumber = req.body.simulationNumber;
  const simulation = username + "_" + simulationNumber;
  const simulationName = req.body.simulationName || `Simulation_${simulationNumber}`; 
  const simulationField = `${username}_${simulationNumber}`;
  const queryInsertSimulation = 'INSERT INTO simulation_queue (username, simulationNumber, simulation, simulationName) VALUES (?, ?, ?, ?)';

  const innerCadFilePaths = req.files.innerCadFiles.map(file => file.path);
  const outerCadFilePaths = req.files.outerCadFiles.map(file => file.path);
  const windFilePaths = req.files.windFiles.map(file => file.path);

  // envia os dados da simulação para o volume
  const sentToVolume = sendToVolume(username, simulationNumber, innerCadFilePaths, outerCadFilePaths, windFilePaths);
  let querryError = 0;
   pool.query(queryInsertSimulation, [username, simulationNumber, simulationField, simulationName], async (error, results) => {
    if (error) {
      querryError = error;
      console.error('Database Insert Error:', error);
    }

    // Now, if the insert is successful, proceed with the rest of the logic (preprocessing)
    try {
      // Call the preprocessing step
      const response = await axios.get('http://simulation:8000/start_preprocessing', {
        params: {
          simulation: simulation
        }
      });

      if (querryError !== 0){
        res.status(200).send(querryError)
      }else{
         // Successfully triggered preprocessing, send success response
        res.status(200).send('Files uploaded and preprocessing started successfully.');
      }
    } catch (error) {
      console.error('Preprocessing Request Error:', error);
      res.status(500).send('Failed to start preprocessing.');
    }
  });
});


app.get('/getSimulationStatus', (req, res) => {
  const simulation = req.query.simulation;

  if (!simulation) {
    return res.status(400).json({ error: 'Simulation is required' });
  }

  const queryGetSimulationStatus = 'SELECT status FROM simulation_queue WHERE simulation = ?';

  pool.query(queryGetSimulationStatus, [simulation], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).json({ error: 'Internal Server Error' });
    }

    if (results.length === 0) {
      return res.status(404).json({ message: 'Simulation not found' });
    }

    const status = results[0].status;
    res.status(200).json({ status });
  });
});

// rota que dá o novo número de simulação 
app.get('/getSimulationNumber', (req, res) => {
  const username = req.query.username;

  if (!username) {
    return res.status(400).json({ error: 'Username is required' });
  }

  const queryGetSimulationNumber = 'SELECT MAX(simulationNumber) AS maxSimulationNumber FROM simulation_queue WHERE username = ?';

  pool.query(queryGetSimulationNumber, [username], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).json({ error: 'Internal Server Error' });
    }

    const maxSimulationNumber = results[0].maxSimulationNumber || 0;
    res.status(200).json({ simulationNumber: maxSimulationNumber + 1 });
  });
});

// rota que dá o id da próxima simulação
app.get('/getFirstInQueue', (req, res) => {
  const queryGetFirstInQueue = 'SELECT * FROM simulation_queue WHERE status = "IN QUEUE" ORDER BY id ASC LIMIT 1';
  
  pool.query(queryGetFirstInQueue, (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.length === 0) {
      return res.status(404).send('No simulations in queue');
    }

    // Return the first record with status "IN QUEUE"
    res.status(200).json({ simulation: results[0].simulation });
  });
});


async function setStatusToInSimulation(simulation) {
  return new Promise((resolve, reject) => {
    const querySetStatus = 'UPDATE simulation_queue SET status = "IN SIMULATION" WHERE simulation = ?';

    pool.query(querySetStatus, [simulation], (error, results) => {
      if (error) {
        console.error('Error updating simulation status:', error);
        return reject('Internal Server Error');
      }

      if (results.affectedRows === 0) {
        return reject('Simulation not found');
      }

      resolve('Status updated successfully');
    });
  });
}

app.post('/setStatusToDone', (req, res) => {
  const simulation = req.body.simulation;
  const querySetStatus = 'UPDATE simulation_queue SET status = "DONE" WHERE simulation = ?';

  pool.query(querySetStatus, [simulation], async(error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    if (results.length === 0) {
      return res.status(404).send('Simulation not found');
    }
    res.status(200).send('Status updated successfully');
  });
});

// User login route
app.post('/login', async (req, res) => {
  const { username, password } = req.body;

  if (!username || !password) {
    return res.status(400).send('Username and password are required');
  }

  try {
    const queryFindUser = 'SELECT password FROM utilizadores WHERE username = ?';

    pool.query(queryFindUser, [username], async (error, results) => {
      if (error) {
        console.error(error);
        return res.status(500).send('Internal Server Error');
      }

      if (results.length === 0) {
        return res.status(404).send('User not found');
      }

      const hashedPassword = results[0].password;
      const match = await bcrypt.compare(password, hashedPassword);

      if (!match) {
        return res.status(401).send('Invalid username or password');
      }

      const token = jwt.sign({ username }, secretKey, { expiresIn: '1h' });
      res.status(200).send({ message: 'Login successful', token });
    });
  } catch (err) {
    console.error(err);
    res.status(500).send('Internal Server Error');
  }
});

// User registration route
app.post('/addUser', async (req, res) => {
  const { username, password } = req.body;

  if (!username || !password) {
    return res.status(400).send('Username and password are required');
  }

  try {
    const hashedPassword = await bcrypt.hash(password, saltRounds);
    const query = 'INSERT INTO utilizadores (username, password) VALUES (?, ?)';

    pool.query(query, [username, hashedPassword], (error, results) => {
      if (error) {
        if (error.code === 'ER_DUP_ENTRY') {
          return res.status(409).send('O username já existe');
        }
        console.error(error);
        return res.status(500).send('Internal Server Error');
      }

      const token = jwt.sign({ username }, secretKey, { expiresIn: '1h' });
      res.status(201).send({ id: results.insertId, username, token });
    });
  } catch (err) {
    console.error(err);
    res.status(500).send('Internal Server Error');
  }
});

//delete user route
app.delete('/deleteUser', async (req, res) => {
  const token = req.headers.authorization?.split(' ')[1];

  if (!token) {
    return res.status(401).send('Access Denied');
  }

  try {
    const decoded = jwt.verify(token, secretKey);
    const username = decoded.username;

    const queryDeleteUser = 'DELETE FROM utilizadores WHERE username = ?';

    pool.query(queryDeleteUser, [username], (error, results) => {
      if (error) {
        console.error(error);
        return res.status(500).send('Internal Server Error');
      }

      res.status(200).send('User deleted successfully');
    });
  } catch (err) {
    console.error(err);
    res.status(500).send('Internal Server Error');
  }
});

// Change password route
app.post('/changePassword', async (req, res) => {
  const { username, currentPassword, newPassword } = req.body;

  if (!username || !currentPassword || !newPassword) {
    return res.status(400).send('Username, current password, and new password are required');
  }

  try {
    const queryFindUser = 'SELECT password FROM utilizadores WHERE username = ?';

    pool.query(queryFindUser, [username], async (error, results) => {
      if (error) {
        console.error(error);
        return res.status(500).send('Internal Server Error');
      }

      if (results.length === 0) {
        return res.status(404).send('User not found');
      }

      const hashedPassword = results[0].password;

      const match = await bcrypt.compare(currentPassword, hashedPassword);

      if (!match) {
        return res.status(401).send('Invalid current password');
      }

      const newHashedPassword = await bcrypt.hash(newPassword, saltRounds);

      const queryUpdatePassword = 'UPDATE utilizadores SET password = ? WHERE username = ?';

      pool.query(queryUpdatePassword, [newHashedPassword, username], (error, results) => {
        if (error) {
          console.error(error);
          return res.status(500).send('Internal Server Error');
        }

        res.status(200).send('Password changed successfully');
      });
    });
  } catch (err) {
    console.error(err);
    res.status(500).send('Internal Server Error');
  }
});

app.listen(port, () => {
  console.log(`Server running on http://localhost:${port}`);
});
