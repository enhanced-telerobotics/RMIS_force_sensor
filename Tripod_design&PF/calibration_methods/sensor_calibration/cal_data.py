import numpy as np

class calData:
    def __init__(self, data):
        self.Time = data.iloc[:, 0].values.astype(float)
        self.ForceData = data.iloc[:, 1:4].values.astype(float)
        self.oriSensor = data.iloc[:, 4:10].values.astype(float)
        self.SensorData = np.hstack((self.oriSensor, np.ones((self.oriSensor.shape[0], 1))))

    def perform_cal(self):
        return np.linalg.lstsq(self.SensorData, self.ForceData, rcond=None)[0]

    def test_cal(self, calMat):
        return np.dot(self.SensorData, calMat)