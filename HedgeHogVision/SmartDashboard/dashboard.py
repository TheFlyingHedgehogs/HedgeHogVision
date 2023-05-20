from networktables import NetworkTables
from networktables import NetworkTable

class VisionNetworkTable():
    @staticmethod
    def __getTable(directory: str) ->  NetworkTable:
        directory = directory.split("/")
        table = NetworkTables.getTable(directory[0])
        if(len(directory) <= 1): return table
        for i in range(1,len(directory)):
            table = table.getSubTable(directory[i])
        return table
    def __init__(self, server: str, visionTable: NetworkTable, odometryTable: NetworkTable):
        NetworkTables.initialize(server="10.28.98.2")

        SmartDashboard = NetworkTables.getTable("Vision")
        OdometryDashboard = NetworkTables.getTable("SmartDashboard").getSubTable("Odometry")
    @staticmethod
    def fromDirectory(server: str, visionTable: NetworkTable, odometryTable: NetworkTable):
        return VisionNetworkTable(server, VisionNetworkTable.__getTable(visionTable))
