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

const app = express();
const port =3000;
const saltRounds = 10;
const secretKey = '8711aff2c33a868742a1122a185e1ecb6b17beb1a4c4097a9cc37092b8fddb20fb5049c81b07dcbf979e678312c039d1939824efbefebdacf94bac0ba1421f5';

app.use(cors());
app.use(bodyParser.json());

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

    if (file.fieldname === 'cadFiles') {
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
function sendToVolume(username, simulationNumber, cadFilePaths, windFilePaths) {
  const baseDir = path.join('/simulation_data', username, `sim_${simulationNumber}`);
    
    // define os subdiretórios
    const cadDir = path.join(baseDir, 'cad_models');
    const windDir = path.join(baseDir, 'wind_simulations');

    // cria os diretórios caso eles não existam
    [cadDir, windDir].forEach(dir => {
        if (!fs.existsSync(dir)) {
            fs.mkdirSync(dir, { recursive: true });
        }
    });

    // copia os ficheiros CAD para o local onde o gaden vai ler
    cadFilePaths.forEach((cadFilePath) => {
        const fileName = path.basename(cadFilePath);
        const destPath = path.join(cadDir, fileName);
        fs.copyFileSync(cadFilePath, destPath);
    });

    // copia a simulação de vento para o local onde o gaden vai ler
    windFilePaths.forEach((windFilePath) => {
      const fileName = path.basename(windFilePath); 
      const windDestPath = path.join(windDir, fileName); 
      fs.copyFileSync(windFilePath, windDestPath);
    });

}

// rota de upload de ficheiros
app.post('/uploadFiles', upload.fields([{ name: 'cadFiles' }, { name: 'windFiles', maxCount: 50 }]), (req, res) => {
  if (!req.files || (!req.files.cadFiles || req.files.cadFiles.length === 0) || (!req.files.windFiles || req.files.windFiles.length === 0)) {
    return res.status(400).send('No files were uploaded.');
  }

  const username = req.body.username;
  const simulationNumber = req.body.simulationNumber;
  const simulationName = req.body.simulationName || `Simulation_${simulationNumber}`; 
  const simulationField = `${username}_${simulationNumber}`;
  const queryInsertSimulation = 'INSERT INTO simulation_queue (username, simulationNumber, simulation, simulationName) VALUES (?, ?, ?, ?)';

  const cadFilePaths = req.files.cadFiles.map(file => file.path);
  const windFilePaths = req.files.windFiles.map(file => file.path);

  // envia os dados da simulação para o volume
  const sentToVolume = sendToVolume(username, simulationNumber, cadFilePaths, windFilePaths);

  // insere os dados da simulação para a base de dados
  pool.query(queryInsertSimulation, [username, simulationNumber, simulationField, simulationName], (error, results) => {
    if (error) {
      console.error(error);
      return res.status(500).send('Internal Server Error');
    }

    res.status(200).send('Files uploaded and launch file created successfully.');
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
