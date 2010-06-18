import unittest
from ompl.util import *

class TestRNG(unittest.TestCase):
	def testDifferentSeeds(self):
		r = [ RNG() for i in range(4) ]
		same = 0
		eq = 0
		N = 100
		for i in range(N):
			v = [ r[j].uniformInt(0,100) for j in range(4) ]
			if v[0]==v[1] and v[1]==v[2] and v[2]==v[3] and v[3]==v[4]:
				eq=eq+1
			for j in range(4):
				if v[j]==r[j].uniformInt(0,100):
					same=same+1
		self.assertFalse(eq > N/2)
		self.assertTrue(same<2*N)
	
	def testValidRangeInts(self):
		r=RNG()
		N=100
		V=10000*N
		c=[0 for i in range(N+1)]
		for i in range(V):
			v = r.uniformInt(0, N)
			self.assertTrue(v>=0)
			self.assertTrue(v<=N)
			c[v]=c[v]+1
		for j in c:
			self.assertTrue(j>V/N/3)
			
def suite():
	suites = (unittest.makeSuite(TestRNG,'test'))
	return unittest.TestSuite(suites)
