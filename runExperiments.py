import subprocess as sp

for noiseIndex in range(1,4):
	
	for testIndex in range(1,7):
		
		testString = 'testdata' + str(testIndex)
		noiseString = 'noiseParameters' + str(noiseIndex)
		
		commandString = 'python gtsamSimulation.py ' + testString + ' ' + noiseString
		
		print commandString
		
		sp.call('python gtsamSimulation.py ' + testString + ' ' + noiseString)
		
		print 'COMPLETED TEST ' + ' ' + testString + ' ' + noiseString


