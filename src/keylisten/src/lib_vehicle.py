import time
import numpy as np
#import struct
#from scipy import interpolate

class Integral():
    def __init__(self):
        self.init(0)
    
    def init(self, y0):
        self.tlast = time.time()
        self.y = y0
    
    def run(self, x):
        self.y = self.y + x*(time.time() - self.tlast)
        self.tlast = time.time()
    def __del__(self):
        print('Destructor Int called')

		
class Lag1stOrder():
    def __init__(self):
        self.init(1, 0)
    
    def init(self, tau, y0):
        self.tlast = time.time()
        self.y = y0
        self.tau = tau
    
    def run(self, x):
        self.y = self.y + (x - self.y)*(time.time() - self.tlast)/self.tau
        #self.y = self.y + x*(time.time() - self.tlast)
        self.tlast = time.time()
    def __del__(self):
        print('Destructor Int called')
		
class Derivative_Spaced():
    def __init__(self):
        self.init(0)
    
    def init(self, x):
        self.tlast = time.time()
        self.y = 0
        self.xlast = x
    
    def run(self, x, dt):
        self.time_elapsed = time.time() - self.tlast
        if self.time_elapsed >= dt:
            self.y = (x - self.xlast) / self.time_elapsed
            self.tlast = time.time()
            self.xlast = x
    def __del__(self):
        print('Destructor called')
			
class Vehicle_Sim():
	def __init__(self):
		#self.axIntegral = Integral()
		self.yrIntegral = Integral()
		self.yaw = self.yrIntegral.y
		self.vnIntegral = Integral()
		self.north = self.vnIntegral.y
		self.veIntegral = Integral()
		self.east = self.veIntegral.y
		self.vxRateLim = Rate_Limit()
		self.curvatureRateLim = Rate_Limit()
		self.init(0, 0, 0)
		#
		# Below parameters should probably get initialized by reading from some param file
		#
		self.wheelbase = 2.7 # Wheelbase
		self.steer_ratio = 18 # Steer Ratio
		self.aymax = 6 # Max Lat. Accel.
		self.g = 9.81 # Gravity in m/s^2
		
	def init(self, east, north, yaw):
		self.running = False
		self.vx = self.vxRateLim.init(0.0)
		self.yaw = yaw # Yaw Angle deg
		self.east = east # East m
		self.north = north # North m
		self.ax = 0 # Long. Accel. m/s^2
		self.yr = 0 # Yaw Rate deg/s
		self.yrIntegral.init(self.yaw)
		self.yaw = self.yrIntegral.y
		self.vnIntegral.init(self.north)
		self.north = self.vnIntegral.y
		self.veIntegral.init(self.east)
		self.east = self.veIntegral.y
    
	#
	def run(self, speed_cmd, accel_cmd, decel_cmd, curvature_cmd, curvature_rate):
		if self.running == False:
			self.running = True
			self.ay = 0.0
			self.tlast = time.time()

		else:
			if speed_cmd >= self.vx:
				self.ax = accel_cmd
			else:
				self.ax = decel_cmd
			self.vxRateLim.run(0.0, self.ax, speed_cmd)
			self.vx = self.vxRateLim.y
			
			# lateral

			self.curvatureRateLim.run(0.0, curvature_rate, curvature_cmd)
			curvature = self.curvatureRateLim.y
			steer = curvature*self.wheelbase*self.steer_ratio*180/np.pi

			self.ay = self.g*min(1-np.exp(-self.vx**2*np.absolute(steer)*np.pi/180/self.steer_ratio/self.wheelbase/self.g), self.aymax/self.g)*np.sign(steer)
			
			if self.vx == 0:
				self.yr = 0
			else:
				self.yr = self.ay/self.vx*180/np.pi
			self.yrIntegral.run(self.yr)
			self.yaw = self.yrIntegral.y
			self.yaw = self.yaw % 360
			
			self.vnIntegral.run(self.vx*np.cos(self.yaw*np.pi/180))
			self.north = self.vnIntegral.y
			self.veIntegral.run(self.vx*np.sin(self.yaw*np.pi/180))
			self.east = self.veIntegral.y
			
			self.tlast = time.time()
			
	def __del__(self):
		print('Destructor called')
		
class Rate_Limit():
	def __init__(self):
		self.init(0)
    
	def init(self, y0):
		self.running = False
		self.y = y0
    
	def run(self, y0, dydt, y):
		if self.running == False:
			self.running = True
			self.y = y0
			self.tlast = time.time()
		else:
			sign = (1 if y - self.y >= 0 else -1)
			dy = min(abs(y - self.y), dydt*(time.time() - self.tlast))
			self.y = self.y + sign*dy
			self.tlast = time.time()
	def __del__(self):
		print('Destructor called')

class Interp():
	def __init__(self):
		self.tlast = time.time()
		self.running = False
		self.y = 0
		self.t = 0
    
	def init(self, data):
		self.tlast = time.time()
		self.running = False
		self.xp = np.fromstring(data.split("%")[0], dtype=float, sep=',')
		self.yp = np.fromstring(data.split("%")[1], dtype=float, sep=',')
		self.y = 0
		self.t = 0

	def run(self):
		if self.running == False:
			self.running = True
			self.tlast = time.time()
		self.t = time.time() - self.tlast
		self.y = np.interp(self.t, self.xp, self.yp)

#
# This function is to convert double to any abrbitrary precision signed or unsigned integer
#		
def double2bytes(input, scale, nBits):
	if input < 0:
		bitStr = "{0:b}".format(int(input*scale + 2 ** (nBits-1)))
		if (len(bitStr) < nBits):
			bitStr = "0" * (nBits-len(bitStr)) + bitStr
		bitList = list(bitStr)
		bitList[0] = '1'
		bitStr = "".join(bitList)
	else:
		bitStr = "{0:b}".format(int(input*scale))
		if (len(bitStr) < nBits):
			bitStr = "0" * (nBits-len(bitStr)) + bitStr
	return struct.pack('I', int(bitStr, 2))[0:divmod(nBits,8)[0]]		
