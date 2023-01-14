from networktables import NetworkTables
NetworkTables.initialize(server="10.28.98.2")
SmartDashboard = NetworkTables.getTable("vision")
