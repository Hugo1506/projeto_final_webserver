-- MySQL dump 10.13  Distrib 8.0.41, for Linux (x86_64)
--
-- Host: localhost    Database: projeto_final
-- ------------------------------------------------------
-- Server version	8.0.41-0ubuntu0.22.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `simulation_queue`
--

DROP TABLE IF EXISTS `simulation_queue`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `simulation_queue` (
  `simulation` varchar(255) NOT NULL,
  `username` varchar(255) NOT NULL,
  `simulationNumber` int NOT NULL,
  PRIMARY KEY (`simulation`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `simulation_queue`
--

LOCK TABLES `simulation_queue` WRITE;
/*!40000 ALTER TABLE `simulation_queue` DISABLE KEYS */;
INSERT INTO `simulation_queue` VALUES ('new_1','new',1),('new_10','new',10),('new_11','new',11),('new_12','new',12),('new_13','new',13),('new_14','new',14),('new_15','new',15),('new_16','new',16),('new_17','new',17),('new_18','new',18),('new_19','new',19),('new_2','new',2),('new_20','new',20),('new_21','new',21),('new_22','new',22),('new_23','new',23),('new_24','new',24),('new_25','new',25),('new_26','new',26),('new_27','new',27),('new_28','new',28),('new_29','new',29),('new_3','new',3),('new_30','new',30),('new_31','new',31),('new_32','new',32),('new_33','new',33),('new_34','new',34),('new_35','new',35),('new_36','new',36),('new_37','new',37),('new_38','new',38),('new_39','new',39),('new_4','new',4),('new_40','new',40),('new_41','new',41),('new_5','new',5),('new_6','new',6),('new_7','new',7),('new_8','new',8),('new_9','new',9),('newr_1','newr',1),('newr_2','newr',2);
/*!40000 ALTER TABLE `simulation_queue` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `utilizadores`
--

DROP TABLE IF EXISTS `utilizadores`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `utilizadores` (
  `id` int NOT NULL AUTO_INCREMENT,
  `username` varchar(255) NOT NULL,
  `password` varchar(100) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE KEY `username` (`username`)
) ENGINE=InnoDB AUTO_INCREMENT=35 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `utilizadores`
--

LOCK TABLES `utilizadores` WRITE;
/*!40000 ALTER TABLE `utilizadores` DISABLE KEYS */;
INSERT INTO `utilizadores` VALUES (7,'testuser','testpassword'),(20,'testuser2','$2b$10$HiJoJdq1HumpRiGkWHGC0e.BsQfcho3LzZUmlf7jAQj2dM3vfgiqO'),(21,'testuser3','$2b$10$k1/iNzDWdURoJxwj.n8o0eXumZAOqdu8kYzi9H7ztW/QhkwAnhD8O'),(22,'test','$2b$10$l9CwTAWfIVBiCFkMpVUE.O.KT9BnjOSP4aDOPm0zg/8XjULrfcqLy'),(23,'testenovo','$2b$10$bsvZOoqm8RPVMNRAE5xSBu8NHdVKAwERBd.L80luqjWLatDYmJYf2'),(24,'autologin','$2b$10$FWIZ4BU4LSG7HUN7Hun45..MDFM2AZ7Bwuc.qqGQYx.7KiiBAoUaW'),(26,'new','$2b$10$CSyKLqKcf41aFeP8vR2O0eHF1XtHJXZUhlQtuxyW.YkC34ezUVHhK'),(27,'testrjwt','$2b$10$hTKJW9Mm/YjUXTSIKw7rZ.JTKpwrQre28RbTmuG1sC64o4js85mku'),(32,'newr','$2b$10$oEUdSzDnZeqRJxhd/UZNKO2fjhHgj6I3RSbGNKrQz3Z8LlA/jNk42'),(34,'newr2','$2b$10$ZcHicskGz6vwSyY/h4qc5.VVDZSUTHC4Ud.SzdnTQgGx.JluZvwdO');
/*!40000 ALTER TABLE `utilizadores` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-03-12 13:56:51
